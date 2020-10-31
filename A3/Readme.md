First Name: Nicola

Last Name: Küng

### Solution to Question 4: 
_Note: I tried to do something really fancy for you here by including latex equations in a hacky way. Now that I realized github seems to be somewhat down and not loading "githubusercontent" anymore, I am also including it as a pdf [here](https://github.com/cmm-20/a3-nkueng/blob/master/q4_answer.pdf). Sorry for the circumstances._

Due to linearity of the trace operator

![\text{tr}(\mathcal{F}^T\mathcal{F} - \mathbf{I}_{2\times 2}) = \text{tr}(\mathcal{F}^T\mathcal{F}) - \text{tr}(\mathbf{I}_{2\times 2}) = \text{tr}(\mathcal{F}^T\mathcal{F}) - 2](https://render.githubusercontent.com/render/math?math=%5Ctext%7Btr%7D(%5Cmathcal%7BF%7D%5ET%5Cmathcal%7BF%7D%20-%20%5Cmathbf%7BI%7D_%7B2%5Ctimes%202%7D)%20%3D%20%5Ctext%7Btr%7D(%5Cmathcal%7BF%7D%5ET%5Cmathcal%7BF%7D)%20-%20%5Ctext%7Btr%7D(%5Cmathbf%7BI%7D_%7B2%5Ctimes%202%7D)%20%3D%20%5Ctext%7Btr%7D(%5Cmathcal%7BF%7D%5ET%5Cmathcal%7BF%7D)%20-%202)

with the definition of matrix multiplication

![\text{tr}(\mathcal{F}^T\mathcal{F}) = \sum_{i}\left\[\mathcal{F}^{T} \mathcal{F}\right\]_{i i}=\sum_{i}\left(\sum_{j} \mathcal{F}_{i j}^{T} \mathcal{F}_{j i}\right)=\sum_{i, j} \mathcal{F}_{i j}^{2}](https://render.githubusercontent.com/render/math?math=%5Ctext%7Btr%7D(%5Cmathcal%7BF%7D%5ET%5Cmathcal%7BF%7D)%20%3D%20%5Csum_%7Bi%7D%5Cleft%5B%5Cmathcal%7BF%7D%5E%7BT%7D%20%5Cmathcal%7BF%7D%5Cright%5D_%7Bi%20i%7D%3D%5Csum_%7Bi%7D%5Cleft(%5Csum_%7Bj%7D%20%5Cmathcal%7BF%7D_%7Bi%20j%7D%5E%7BT%7D%20%5Cmathcal%7BF%7D_%7Bj%20i%7D%5Cright)%3D%5Csum_%7Bi%2C%20j%7D%20%5Cmathcal%7BF%7D_%7Bi%20j%7D%5E%7B2%7D)

employing the definition of the Frobenius norm

![\sum_{i, j} \mathcal{F}_{i j}^{2} = \| \mathcal{F}\|^2_F](https://render.githubusercontent.com/render/math?math=%5Csum_%7Bi%2C%20j%7D%20%5Cmathcal%7BF%7D_%7Bi%20j%7D%5E%7B2%7D%20%3D%20%5C%7C%20%5Cmathcal%7BF%7D%5C%7C%5E2_F)

we arrive at the desired result

![\text{tr}(\mathcal{F}^T\mathcal{F} - \mathbf{I}_{2\times 2}) = \| \mathcal{F}\|^2_F - 2](https://render.githubusercontent.com/render/math?math=%5Ctext%7Btr%7D(%5Cmathcal%7BF%7D%5ET%5Cmathcal%7BF%7D%20-%20%5Cmathbf%7BI%7D_%7B2%5Ctimes%202%7D)%20%3D%20%5C%7C%20%5Cmathcal%7BF%7D%5C%7C%5E2_F%20-%202)

### Solution to Question 10:
Intuitively speaking, the optimizer finds the first solution (with twisted ends) since that configuration is **closer** to the original configuration. Getting to the second solution (with straight ends) means going further away from the original configuration, not only in terms of translations `x` and `y`, but also `theta`. Thus, when our gradient descent emerges on its quest to find a close-by minimum (in the parameter space spanned by `x`, `y`, and `theta`), it will first find encounter the minimum which is associated with smaller changes to `u`. (This holds in general, such that the simulation never really changes the angles of the handles since that means larger changes to `u`.)

Now, there exist a whole bunch of valid solutions in-between the two shown configurations that fulfill the objective function (approximately) equally well. Let's refer to all those solution-configurations as the *nullspace*. Inside of this nullspace, there exists a configuration with straight ends, but our optimizer doesn't find it (usually) since it's no better at fulfilling the objective at hand. Only when we add another term (i.e., a regularizer) to the objective function will its "landscape" change in a way that the second configuration (straight ends) fulfills the updated objective better than the first configuration (twisted ends).

### Solution to Question 11:
There exist many ways to express *mathematically* how the desired second configuration deviates from the undesired first one. The question boils down to "what else should the optimizer minimize apart from the distance to the target position?". My solution adds as a sub-objective the (squared) angular deviation between the angles of the handles (`theta_L` and `theta_R`) and the angle of the line connecting the target positions of the feature points (simply `angle`):
```
O += pow(10, lambda) * (pow(theta_L - angle, 2) + pow(theta_R - angle, 2));
```
This way, the optimizer pays attention to both, finding a configuration that tracks the target position as well as making the ends parallel to the line connecting the feature points (usually parallel to the overall orientation of the beam). The value of `-1` for `lambda` offers a good trade-off between the two objectives.

---

Assignment writeup: http://crl.ethz.ch/teaching/computational-motion-20/slides/tutorial-a3.pdf

---

1. Clone this repository.
    ```
    git clone YOUR_GIT_URL
    ```
2. Make build folder and run cmake
    ```
    cd a3
    mkdir build && cd build
    cmake ..
    ```
    - NOTE: Windows users can also try double-clicking runCMakeVisualStudio.cmd
    - NOTE: Don't forget to switch to release mode or it's gonna be real slow.
3. Compile code and run executable
    - for Windows: 
        * open `a3.sln` in Visual Studio
        * in the project explorer, right-click target "fem-app" or "manip-app" and set as startup app.
        * Hit F5 or press that Play Button :)
    - for MacOS and Linux:
        ```
        make
        ./src/app/app
        ```
