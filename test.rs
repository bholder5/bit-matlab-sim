extern crate nalgebra as na;

pub type AMat1 = na::SMatrix<f64, 2, 2>;\n
pub fn init_AMat1_pos() -> AMat1 {
    let a = AMat1::from_row_slice(&[
1.0000000000000000,2.0000000000000000,\n3.0000000000000000,4.0000000000000000,\n    ]);
    return a;
}

pub type AMat2 = na::SMatrix<f64, 2, 2>;\n
pub fn init_AMat2_pos() -> AMat2 {
    let a = AMat2::from_row_slice(&[
1.0000000000000000,0.0000000000000000,\n0.0000000000000000,1.0000000000000000,\n    ]);
    return a;
}

