weight_data = 0.5   # How much weight to update the data (a)
weight_smooth = 0.1 # How much weight to smooth the coordinates (b)
min_change = 0.0001 # Minimum change per iteration to keep iterating
new_path = np.copy(np.array(self.path))
path_length = len(self.path[0])
change = min_change

while change >= min_change:
    change = 0.0
    for i in range(1, len(new_path)-1):
        for j in range(path_length):
            # Initialize x, y
            x_i = self.path[i][j]
            y_i = new_path[i][j]
            y_prev = new_path[i-1][j]
            y_next = new_path[i+1][j]

            y_i_saved = y_i

            # Minimize the distance between coordinates of the original
            # path (y) and the smoothed path (x). Also minimize the 
            # difference between the coordinates of the smoothed path 
            # at time step i, and neighboring time steps. In order to do
            # all the minimizations, we use Gradient Ascent.
            y_i += weight_data * (x_i-y_i) + weight_smooth * (y_next + y_prev - (2*y_i))
            new_path[i][j] = y_i

            change += abs(y_i - y_i_saved)

self.path = new_path