use ode_solvers::dopri5::*;
use ode_solvers::*;
use std::{fs::File, io::Write, path::Path};

type State = Vector4<f64>;
type Time = f64;

struct seirModel {
    recoveryRate: f64,
    reproductionNumber: f64,
    infectionRate: f64,
}

impl ode_solvers::System<State> for seirModel {
    fn system(&self, _t: Time, y: &State, dy: &mut State){
        let s = y[0];
        let e = y[1];
        let i = y[2];
        let r = y[3];

        dy[0] = -self.recoveryRate*self.reproductionNumber*s*i;
        dy[1] = self.recoveryRate*self.reproductionNumber*s*i - self.infectionRate*e;
        dy[2] = self.infectionRate*e - self.recoveryRate*i;
        dy[3] = self.recoveryRate*i;
    }
}

fn main() {
    let system = seirModel {recoveryRate:0.0555,reproductionNumber:3.0,infectionRate:0.1923076923};
    let i0 = 1.0e-7;
    let e0 = 4.0 * i0;
    let s0 = 1.0 - i0 - e0;
    let r0 = 0.0;
    let y0 = State::new(s0,e0,i0,r0);

    let mut stepper = Dopri5::new(system,0.0,350.0, 1.0,y0 ,1.0e-10,1.0e-10);
    let res = stepper.integrate();

    match res {
        Ok(stats) => {
            println!("{}", stats);
            let path = Path::new("/home/karthiksundar/Documents/seirModel_rk4.dat");
            save(stepper.x_out(), stepper.y_out(), path);
            println!("Results saved in: {:?}", path);



        },
        Err(_) => println!("An error occured."),
    }
}


pub fn save(times: &Vec<Time>, states: &Vec<State>, filename: &Path) {
    // Create or open file
    let mut buf = match File::create(filename) {
        Err(e) => {
            println!("Could not open file. Error: {:?}", e);
            return;
        }
        Ok(buf) => buf,
    };

    // Write time and state vector in a csv format
    for (i, state) in states.iter().enumerate() {
        buf.write_fmt(format_args!("{}", times[i])).unwrap();
        for val in state.iter() {
            buf.write_fmt(format_args!(", {}", val)).unwrap();
        }
        buf.write_fmt(format_args!("\n")).unwrap();
    }
}
