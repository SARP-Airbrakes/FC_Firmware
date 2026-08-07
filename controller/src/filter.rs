use nalgebra::{SVector, Unit};

pub type FilterState = SVector<f32, 16>;
pub type Quaternion = nalgebra::Quaternion<f32>;
pub type Vector3 = nalgebra::Vector3<f32>;
pub type Vector4 = nalgebra::Vector4<f32>;
pub type Matrix3 = nalgebra::Matrix3<f32>;

fn skew_symmetric(vec: Vector3) -> Matrix3 {
	Matrix3::new(0.0   , -vec.z, vec.y ,
                 vec.z , 0.0   , -vec.x,
                 -vec.y, vec.x , 0.0   )
}

fn rotation_matrix_from_quat(quat: Quaternion) -> Matrix3 {
    let mut comp = Vector3::zeros();
    let w = quat.w;
    {
        let comp_view = quat.coords.view((1, 0), (3, 1));
        comp.copy_from(&comp_view);
    }
    2.0 * comp * comp.transpose() 
        + Matrix3::identity() * ((w * w) - (comp.transpose() * comp).to_scalar()) 
        - 2.0 * w * skew_symmetric(comp) 
}

pub struct Filter {
    error_state: FilterState,
}

impl Filter {

    pub fn new() -> Self {
        Filter {
            error_state: FilterState::zeros()
        }
    }

    pub fn rotation_error_quat(&self) -> Quaternion {
        Quaternion::new(1.0, self.error_state[0], self.error_state[1], self.error_state[2])
    }
}


