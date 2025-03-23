import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Define the systolic array grid dimensions (4x4 for the kernel)
grid_rows, grid_cols = 4, 4

# Create figure and axis
fig, ax = plt.subplots(figsize=(6,6))
ax.set_xlim(0, grid_cols)
ax.set_ylim(0, grid_rows)
ax.set_xticks(np.arange(0, grid_cols+1, 1))
ax.set_yticks(np.arange(0, grid_rows+1, 1))
ax.grid(True)

# Invert y-axis so that (0,0) is at the top-left
ax.invert_yaxis()

# Pre-create text objects for each cell in the grid
cell_texts = [[ax.text(j + 0.5, i + 0.5, "", ha="center", va="center", fontsize=16)
               for j in range(grid_cols)] for i in range(grid_rows)]

# Define the 4x4 input patch (each element represents an activation)
# (This would be one sliding window from your overall 6x6 input patch.)
activations = np.array([["a00", "a01", "a02", "a03"],
                        ["a10", "a11", "a12", "a13"],
                        ["a20", "a21", "a22", "a23"],
                        ["a30", "a31", "a32", "a33"]])

# Total number of frames:
# For a 4x4 grid, a simple diagonal wavefront will require (rows + cols - 1) frames.
total_frames = grid_rows + grid_cols

# To hold a reference to the output arrow (so we can update it)
output_arrow = None

def update(frame):
    global output_arrow
    # Clear texts in all cells each frame
    for i in range(grid_rows):
        for j in range(grid_cols):
            cell_texts[i][j].set_text("")
    
    # For each cell (i, j), if i+j equals the current frame number, show the activation
    for i in range(grid_rows):
        for j in range(grid_cols):
            if i + j == frame and i < activations.shape[0] and j < activations.shape[1]:
                cell_texts[i][j].set_text(activations[i, j])
    
    # Remove previous output annotation, if any
    if output_arrow is not None:
        output_arrow.remove()
        output_arrow = None

    # When the wave reaches the bottom-right cell, simulate the output being shifted out.
    # Here, we assume that at frame = total_frames - 1, the bottom-right cell (3,3) holds the final computed result.
    if frame >= total_frames - 1:
        output_arrow = ax.annotate("Output", 
                                   xy=(grid_cols - 0.5, grid_rows - 0.5), 
                                   xytext=(grid_cols + 0.5, grid_rows - 0.5),
                                   arrowprops=dict(arrowstyle="->", lw=2),
                                   fontsize=16)
    return sum(cell_texts, []) + ([output_arrow] if output_arrow is not None else [])

# Create the animation (1 second per frame for clarity)
ani = animation.FuncAnimation(fig, update, frames=total_frames, interval=1000, blit=True)

plt.title("Systolic Array Dataflow for conv2d (4×4 window)")
plt.tight_layout()
plt.show()
