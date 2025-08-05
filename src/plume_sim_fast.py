import numpy as np
from scipy.spatial.distance import cdist
from scipy.spatial import cKDTree
from dataclasses import dataclass
from numba import jit, njit
import threading
from functools import lru_cache


def logit(x, lower=0.0, upper=10.0, eps=1e-8):
    x_clamped = np.clip(x, lower + eps, upper - eps)
    scale = upper - lower
    ratio = (x_clamped - lower) / scale
    return np.log(ratio / (1 - ratio))

def inv_logit(z, lower=0.0, upper=10.0):
    scale = upper - lower
    return lower + scale / (1.0 + np.exp(-z))

@njit
def update_ar2_in_zspace_numba(z_current, z_prev, z_target, distance, ar1, ar2, 
                               base_noise_scale=0.1, jump_prob=0.03):
    """Optimized numba version of AR(2) update function"""
    distance_factor = np.exp(-distance / 50.0)
    ar1_local = ar1 * (1 + 0.1 * distance_factor)
    ar2_local = ar2 * (1 - 0.1 * distance_factor)
    noise = base_noise_scale * (1 + 2 * distance_factor) * np.random.randn()
    
    # Simplify the jump logic to be more numba-friendly
    jump = 0.0
    if np.random.random() < jump_prob:
        jump = np.random.uniform(-1, 1) * base_noise_scale * 3
    
    z_next = 0.85 * (ar1_local * (z_current - z_target) + ar2_local * (z_prev - z_target)) + z_target + noise + jump
    return z_next

@njit
def update_ar2_concentration_numba(current, prev, target, ar1, ar2, noise_scale):
    """Numba-optimized background AR(2) update"""
    noise = noise_scale * (np.random.randn() - 0.5) * 0.5
    new_val = 0.85 * (ar1 * (current - target) + ar2 * (prev - target)) + target + noise
    return new_val

@dataclass
class OdorConfig:
    rows_per_second: int = 200
    base_odor_level: float = 0.6
    distance_threshold: float = 3
    ar1: float = 0.98
    ar2: float = -0.02
    warmup_steps: int = 100         # No warmup delay (set >0 if desired)
    low_threshold: float = 0.05
    history_length: int = 7
    transition_matrix: np.ndarray = np.array([[0.15, 0.85],
                                              [0.15, 0.85]])

class OdorStateManager:
    def __init__(self, config, whiff_intermittency):
        # Initialize in z-space via the logit of base concentration.
        z_init = logit(config.base_odor_level, 0, 10)
        self.z_current = z_init
        self.z_prev = z_init
        self.current_concentration = config.base_odor_level
        self.prev_concentration = config.base_odor_level
        
        # Use numpy arrays instead of lists for better performance
        self.recent_history = np.zeros(1000, dtype=np.int8)
        self.recent_concentrations = np.full(10, config.base_odor_level)
        self.recent_intermittencies = np.random.choice(whiff_intermittency, 5)
        
        self.in_whiff_state = False
        self.state_duration = 0  # How long we have been in the current whiff

class CosmosFast:
    def __init__(self, fitted_p_heatmap, xedges, yedges, fdf, fdf_nowhiff):
        """
        Parameters:
          - fitted_p_heatmap: the 2D heatmap of whiff probability.
          - xedges, yedges: edges used to bin space.
          - fdf: DataFrame with whiff data (mean concentration, std, duration, intermittency, etc.)
          - fdf_nowhiff: DataFrame with no-whiff background data.
        """
        self.config = OdorConfig()
        self.fitted_p_heatmap = fitted_p_heatmap
        self.xedges = xedges
        self.yedges = yedges
        
        # Pre-compute and store arrays from dataframes
        self.whiff_locations = fdf[['avg_distance_along_streakline','avg_nearest_from_streakline']].values
        self.nowhiff_locations = fdf_nowhiff[['avg_distance_along_streakline','avg_nearest_from_streakline']].values
        self.mean_concentration = fdf.mean_concentration.values
        self.std_whiff = fdf.std_whiff.values
        self.length_of_encounter = fdf.length_of_encounter.values
        self.odor_intermittency = fdf.odor_intermittency.values
        self.wc_nowhiff = fdf_nowhiff.wc_nowhiff.values
        self.wsd_nowhiff = fdf_nowhiff.wsd_nowhiff.values
        
        # Build KD-Tree for faster nearest neighbor searches
        self.whiff_kdtree = cKDTree(self.whiff_locations)
        self.nowhiff_kdtree = cKDTree(self.nowhiff_locations)
        
        # Create the persistent state.
        self.state = OdorStateManager(self.config, self.odor_intermittency)
        self.steps_processed = 0
        
        # For whiff events, store remaining duration.
        self.current_whiff_duration = 0
        self.current_mean = self.config.base_odor_level
        self.current_std = 0.0
        
        # Setup binned data for intermittency generation.
        self.setup_data()
        
        # Cache for get_spatial_prob
        self._prob_cache = {}
        self._cache_lock = threading.Lock()
        
        # Pre-compute constants for performance
        self.ar1 = self.config.ar1
        self.ar2 = self.config.ar2
        self.distance_threshold = self.config.distance_threshold
        self.rows_per_second = self.config.rows_per_second
        self.low_threshold = self.config.low_threshold
        
        # Convolution window for smoothing
        self.window = np.ones(5) / 5.0

    def setup_data(self):
        distance_bins = np.arange(0, 41, 1)
        nearest_bins = np.arange(0, 9, 1)
        self.bin_data_dict = {}
        
        # Pre-compute all bin data at initialization
        for i in range(len(distance_bins)-1):
            for j in range(len(nearest_bins)-1):
                start_dist, end_dist = distance_bins[i], distance_bins[i+1]
                start_near, end_near = nearest_bins[j], nearest_bins[j+1]
                
                # Use boolean indexing for faster filtering
                dist_mask = (self.whiff_locations[:, 0] >= start_dist) & (self.whiff_locations[:, 0] < end_dist)
                near_mask = (self.whiff_locations[:, 1] >= start_near) & (self.whiff_locations[:, 1] < end_near)
                combined_mask = dist_mask & near_mask
                
                if np.any(combined_mask):
                    bin_data = self.odor_intermittency[combined_mask]
                    # Store data as numpy array for better performance
                    if len(bin_data) > 0:
                        self.bin_data_dict[(start_dist, end_dist, start_near, end_near)] = bin_data

    @lru_cache(maxsize=1024)
    def get_spatial_prob(self, x, y):
        """Return the probability from the fitted heatmap given the spatial coordinates, with caching."""
        # Convert to integer indices for better cache hits
        x_idx = min(max(0, np.searchsorted(self.xedges, x) - 1), len(self.xedges) - 2)
        y_idx = min(max(0, np.searchsorted(self.yedges, y) - 1), len(self.yedges) - 2)
        return self.fitted_p_heatmap[x_idx, y_idx]

    def update_whiff_posterior(self, prior_prob, state):
        """Update the whiff transition probability based on recent state."""
        whiff_state = 1 if state.in_whiff_state else 0
        
        # Count recent whiffs efficiently
        num_recent_whiffs = np.sum(state.recent_history[-20:])
        
        # Find time since last whiff using numpy
        if num_recent_whiffs > 0:
            time_since_whiff = np.argmax(state.recent_history[::-1]) 
        else:
            time_since_whiff = len(state.recent_history)
            
        scaler = 0.5  # parameter you can adjust
        time_since_last_whiff = min(1.5, time_since_whiff) if time_since_whiff > 50 else 1.0
        recent_whiff_memory = (1 + num_recent_whiffs * scaler) * time_since_last_whiff
        posterior = ((prior_prob * scaler)
                     * self.config.transition_matrix[whiff_state][1]
                     * recent_whiff_memory)
        return posterior

    def generate_intermittency(self, distance_along, distance_from, state, default=0.05):
        """Return a random intermittency value from the appropriate binned data."""
        # Using numpy operations for better performance
        last_values = state.recent_intermittencies[-self.config.history_length:]
        low_frequency = np.mean(last_values < self.low_threshold)
        
        # Find the appropriate bin
        for (sd, ed, sn, en), values in self.bin_data_dict.items():
            if (sd <= distance_along < ed) and (sn <= distance_from < en):
                if len(values) > 0:
                    if low_frequency > 0.5:
                        median_val = np.median(values)
                        subset = values[values < median_val]
                        intermittency = np.random.choice(subset) if len(subset) > 0 else np.random.choice(values)
                    else:
                        intermittency = np.random.choice(values)
                    return np.clip(intermittency, np.min(values), np.max(values))
        return default

    def step_update(self, x, y, dt=0.005):
        """
        Update odor concentration with more continuous whiff checking and longer durations.
        Optimized version with faster spatial queries and numba acceleration.
        """
        self.steps_processed += 1
        if self.steps_processed < self.config.warmup_steps:
            return self.config.base_odor_level

        pos = np.array([x, y])
        
        # Use KD-Tree for faster nearest neighbor searches instead of cdist
        dist_whiff, nearest_idx = self.whiff_kdtree.query(pos, k=1)
        dist_from_source = np.sqrt(x**2 + y**2)
        
        # Get spatial probability and posterior
        prior_prob = self.get_spatial_prob(x, y)
        posterior = self.update_whiff_posterior(prior_prob, self.state)

        # Check for new whiff opportunity if not in whiff state or near end of current whiff
        should_check_whiff = (
            not self.state.in_whiff_state or 
            self.current_whiff_duration <= 5 or
            dist_whiff <= self.distance_threshold * 0.5  # Check more aggressively when close
        )
        
        if should_check_whiff and dist_whiff <= self.distance_threshold:
            # Higher chance of maintaining/entering whiff state when close to source
            distance_factor = np.exp(-dist_from_source / 20.0)
            transition_prob = posterior * (1 + distance_factor)
            
            if np.random.rand() < transition_prob:
                self.state.in_whiff_state = True
                
                # Set longer duration for whiffs closer to source
                base_duration = self.length_of_encounter[nearest_idx]
                duration_factor = 1 + 2 * distance_factor
                duration = int(base_duration * duration_factor * self.rows_per_second)
                
                # Update whiff parameters
                self.current_mean = self.mean_concentration[nearest_idx]
                self.current_std = self.std_whiff[nearest_idx]
                self.current_whiff_duration = duration
                
                # Calculate intermittency with distance-dependent adjustment
                intermittency = self.generate_intermittency(
                    self.whiff_locations[nearest_idx, 0],
                    self.whiff_locations[nearest_idx, 1],
                    self.state
                )
                # Shorter intermittency periods when closer to source
                intermittency *= (1 - 0.5 * distance_factor)
                
                # Update recent intermittencies (using numpy roll for better performance)
                self.state.recent_intermittencies = np.roll(self.state.recent_intermittencies, -1)
                self.state.recent_intermittencies[-1] = intermittency

        # Generate concentration based on state
        if self.state.in_whiff_state and dist_whiff <= self.distance_threshold * 1.2:
            # Whiff concentration generation
            z_target = logit(self.current_mean, 0, 10)
            
            # Use numba-optimized function
            z_next = update_ar2_in_zspace_numba(
                self.state.z_current,
                self.state.z_prev,
                z_target,
                distance=dist_from_source,
                ar1=self.ar1,
                ar2=self.ar2,
                base_noise_scale=0.15 * self.current_std,
                jump_prob=0.05
            )
            new_concentration = inv_logit(z_next, 0, 10)
            
            # Update states
            self.state.z_prev = self.state.z_current
            self.state.z_current = z_next
            self.state.prev_concentration = self.state.current_concentration
            self.state.current_concentration = new_concentration
            
            # Update histories using numpy operations
            self.state.recent_concentrations = np.roll(self.state.recent_concentrations, -1)
            self.state.recent_concentrations[-1] = new_concentration
            self.state.recent_history = np.roll(self.state.recent_history, -1)
            self.state.recent_history[-1] = 1
            
            # Decrement duration and check for whiff end
            self.current_whiff_duration -= 1
            if self.current_whiff_duration <= 0:
                self.state.in_whiff_state = False
            
            return new_concentration
            
        else:
            # No-whiff background concentration
            _, nearest_idx = self.nowhiff_kdtree.query(pos, k=1)
            no_whiff_mean = self.wc_nowhiff[nearest_idx]
            no_whiff_std = self.wsd_nowhiff[nearest_idx]
            
            # Use numba-optimized function
            new_concentration = update_ar2_concentration_numba(
                self.state.current_concentration,
                self.state.prev_concentration,
                no_whiff_mean,
                self.ar1,
                self.ar2,
                0.05 * no_whiff_std
            )
            new_concentration = np.clip(new_concentration, 0.6, 1.0)
            
            # Smoothing with pre-allocated window
            if self.steps_processed >= 5:
                window_data = np.append(self.state.recent_concentrations[-10:], new_concentration)
                new_concentration = np.convolve(window_data, self.window, mode='valid')[-1]
            
            # Update states
            self.state.prev_concentration = self.state.current_concentration
            self.state.current_concentration = new_concentration
            
            # Update histories using numpy operations
            self.state.recent_concentrations = np.roll(self.state.recent_concentrations, -1)
            self.state.recent_concentrations[-1] = new_concentration
            self.state.recent_history = np.roll(self.state.recent_history, -1)
            self.state.recent_history[-1] = 0
            
            return new_concentration

# Batch processing function to update multiple positions at once
def batch_update_positions(predictor, positions, dt=0.005):
    """Process multiple positions in parallel for better performance."""
    return [predictor.step_update(x, y, dt) for x, y in positions]