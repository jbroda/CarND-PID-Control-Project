### Describe the effect each of the P, I, D components had in your implementation.

P (proporitional component) - This component adjusts the steering angle proportionally to the Cross Track Error (CTE). Value chosen: 0.2

I (intergral component) - This component adjusted the steering angle for the effects of the past CTE. Value chosen: 0.004.

D (derivative component) - This component reduces oscillation and overshoot in the steering angle. Value chosen: 3.00.


### Describe how the final hyperparameters were chosen.

I chose the hyperparameters manually, using the lecture materials as the starting point.
