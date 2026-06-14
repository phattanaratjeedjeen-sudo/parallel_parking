## 1. The Trend Line: Gear Changes vs. Spot LengthThe 
- Goal: To prove that tighter parking spots require more shunting, and that having a higher maximum steering angle reduces the amount of shunting required.

- The Chart: A Line Chart.
    - X-Axis: Spot Length (from 7.2m down to 5.2m).
    - Y-Axis: Number of Gear Changes (or $n_{trials}$).
    - Series (Lines): 3 separate lines representing Max Steer (30°, 35°, 40°).

- What it shows: You will likely see three curves that swoop upward as the spot length decreases. The 30° line will spike upward much earlier than the 40° line, proving that a sharper steering angle is vastly superior for tiny spots.


## 2. The Accuracy Plot: Final Y-Position Error
- The Goal: To show how the parameters affect the final parking accuracy (which directly relates to the Y-axis drifting bug you solved earlier).

- The Chart: A Grouped Bar Chart or Line Chart.
    - X-Axis: Spot Length.
    - Y-Axis: Final Lateral Error ($|y_{final} - y_{target}|$).
    - Series: 3 lines/bars for the Max Steer angles.

- What it shows: If you used the mathematical/symmetric limits, this graph will prove that your controller achieves near-zero error across all 15 cases. If you used the asymmetric limits without the closed-loop feedback, this graph will expose the drift!


## 3. The Performance Heatmap (Great for Papers)
- The Goal: To provide a dense, easy-to-read summary matrix of the entire experiment.

- The Chart: A 2D Heatmap 
    - Grid.Columns: Spot Length (7.2, 6.7, 6.2, 5.7, 5.2).
    - Rows: Max Steer (30, 35, 40).
    - Color/Values: Number of Gear Changes. (Cool colors like blue for 2-4 gear changes, hot colors like red for 10+ gear changes).

- What it shows: At a single glance, a reader can see the "danger zone" (bottom right of the matrix: small spot, low steering angle) vs. the "safe zone" (top left: big spot, high steering angle).


## 4. The Spatial Trajectory Overlay
- The Goal: To visually demonstrate how the car moves differently under different settings.

- The Chart: An X-Y Trajectory Plot (like the one in your plot_results.py).
    - Setup: Choose one specific Spot Length (e.g., the challenging 5.7m spot).
    - Lines: Plot the actual X-Y path of the car for 30°, 35°, and 40° on the same exact graph.

- What it shows: The 40° line will show a tight, sharp "S-curve" that gets into the spot quickly. The 30° line will show a dense, repetitive zig-zag (crab-walking) as it struggles to shift laterally. This is highly visual and impressive for a presentation.


above visualization only for crab-like planning. but my project have another one. human-like
here is how to adapt 

Here is how you adapt those 4 specific graphs to handle the "Method" variable perfectly, along with the complete Python code to generate them.How to Adapt the 4 Visualizations1. The Trend Line: Gear Changes vs. Spot LengthThe Adaptation: You now use Color to represent the Steering Angle (Blue=30°, Orange=35°, Green=40°) and Line Style to represent the Method (Solid Line = Crab-like, Dashed Line = Human-like).The Story: Readers can instantly see if the dashed lines (Human) always sit below the solid lines (Crab), proving that the human heuristic requires less shunting overall.2. The Accuracy Plot: Final Y-Position ErrorThe Adaptation: Same as above. Color for Steer, Line Style for Method.The Story: This will show the trade-off. Does the Human-like method require fewer gear changes but result in slightly worse parking accuracy (higher Y-error)? This graph will expose that compromise.3. The Heatmap: The "A/B Difference" Matrix (Highly Recommended)The Adaptation: Instead of showing the raw number of gear changes, you plot the Difference between the two methods ($\Delta$ Gear Changes = Crab - Human).The Story: A cell value of +4 means the Human method saved 4 gear changes compared to the Crab method. A heatmap showing green for "Human is better" and red for "Crab is better" instantly summarizes which algorithm wins in which environments.4. The Spatial Trajectory Overlay: Head-to-HeadThe Adaptation: Instead of comparing three steering angles, you compare the Two Methods against each other in exactly one highly constrained environment.The Setup: Pick a challenging scenario (e.g., Spot = 5.7m, Steer = 35°). Plot the Crab path (Blue) vs the Human path (Red) on the exact same X-Y grid.The Story: This visually answers why one method is better than the other by showing their literal pathing logic.