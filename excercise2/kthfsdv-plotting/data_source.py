class DataSource:
    """DataSource class"""

    def __init__(self, equation, sample_rate):
        self.equation = equation  # Numpy equation to plot the signal for
        self.sample_rate = sample_rate  # in ms
        self.time_values = [0]  # values for the x-Axis
        self.signal_values = [
            self.equation(self.time_values[0])
        ]  # values for the y-Axis

    def measure(self):
        """Measures the next time point and appends new values to the time and signal lists."""
        new_time = self.time_values[-1] + (
            self.sample_rate / 1000
        )  # add new time step with <<samplerate>> distance to the last.
        self.time_values.append(new_time)
        self.signal_values.append(self.equation(new_time))

    def reset(self):
        """Resets the time and signal values to the initial state."""
        self.time_values = [0]
        self.signal_values = [self.equation(self.time_values[0])]

    # Dunder methods
    def __getitem__(self, key):
        """Allows for indexing to retrieve time and signal pairs."""
        return self.time_values[key], self.signal_values[key]

    def __len__(self):
        # self.time_values and self.signal_values are always updated together so their length is equal.
        assert self.time_values == self.signal_values
        return len(self.time_values)
    
    # Getter methods
    def get_time_values(self):
        return self.time_values
    
    def get_signal_values(self):
        return self.signal_values
