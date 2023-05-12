pub struct Kalman {
    Q_angle: f64,
    Q_bias: f64,
    R_measure: f64,
    angle: f64,
    bias: f64,
    rate: f64,
    P: [[f64; 2]; 2],
    K: [f64; 2],
    y: f64,
    S: f64,
}

impl Kalman {
    pub fn new(
        Q_angle: f64,
        Q_bias: f64,
        R_measure: f64,
        angle: f64,
        bias: f64,
        rate: f64,
        P: [[f64; 2]; 2],
        K: [f64; 2],
        y: f64,
        S: f64,
    ) -> Self {
        Self {
            Q_angle: Q_angle,
            Q_bias: Q_bias,
            R_measure: R_measure,
            angle: angle,
            bias: bias,
            rate: rate,
            P: P,
            K: K,
            y: y,
            S: S,
        }
    }
    pub fn compute_angle(&mut self, new_angle: f64, new_rate: f64, dt: f64) -> f64 {
        /*step 1*/
        self.rate = new_rate - self.bias;
        self.angle += dt * self.rate;

        /*step2 */
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle);
        self.P[0][1] -= dt * self.P[1][1];
        self.P[1][0] -= dt * self.P[1][1];
        self.P[1][1] += self.Q_bias * dt;
        /*step 4 */
        self.S = self.P[0][0] + self.R_measure;
        /*step 5 */
        self.K[0] = self.P[0][0] / self.S;
        self.K[0] = self.P[1][0] / self.S;
        /*step 3 */
        self.y = new_angle - self.angle;
        /*step 6 */
        self.angle += self.K[0] * self.y;
        self.bias += self.K[1] * self.y;
        /*step 7 */
        self.P[0][0] -= self.K[0] * self.P[0][0];
        self.P[0][1] -= self.K[0] * self.P[0][1];
        self.P[1][0] -= self.K[1] * self.P[1][0];
        self.P[1][1] -= self.K[1] * self.P[1][1];

        return self.angle;
    }
}
