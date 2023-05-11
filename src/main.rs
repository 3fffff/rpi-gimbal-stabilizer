mod PID;
use PID::PID as Pid;
fn main() {
    let mut pid = Pid::new(2.0, 0.5, 0.1, 0.0, 255.0, 10, false, false, 100.0);
    let mut input = 1.0;
    let mut out:f64=99.5;
    loop{
        input = out;
        let outr = pid.compute(&input, &mut out);
        println!("{},{}",outr,out);
    }
}
