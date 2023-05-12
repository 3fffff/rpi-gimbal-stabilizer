mod PID;
mod Kalman;
use PID::PID as Pid;
use Kalman::Kalman as kalman;
fn main() {
    let mut pid_x = Pid::new(2.0, 0.5, 0.1, -255.0, 255.0, 10, false,  0.0);
    let mut pid_y = Pid::new(2.0, 0.5, 0.1, -255.0, 255.0, 10, false,  0.0);
    let mut input_x = 1.0;//IMU calibrated angles
    let mut input_y = 0.0;
    let mut out_x:f64=0.0;
    let mut out_y:f64=0.0;
    loop{
        let out_xb = pid_x.compute(&input_x, &mut out_x);
        let out_yb = pid_y.compute(&input_y, &mut out_y);
        let mut x_out = out_x;
        let mut y_out = out_y;
        if x_out<0.0{
            //reverse pin
            x_out = x_out.abs();
        }
        else{// norm pin
        }
        if y_out<0.0{
            y_out = y_out.abs();
        }else{
            //norm pin
        }
        if out_xb{
            //write pin X
        }
        if out_yb{
            //write pin Y
        }
        //println!("{},{}",outr,out);
    }
}
