use zenoh::Session;
use edgefirst_schemas::sensor_msgs::Image;
use edgefirst_schemas::std_msgs::Header;
use edgefirst_schemas::builtin_interfaces::Time;
use serde::{Deserialize, Serialize};
use kinematics::JointState as KinematicsJointState;
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct JointState {
    pub header: Header,
    pub name: Vec<String>,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
    pub effort: Vec<f64>,
}

impl Default for JointState {
    fn default() -> Self {
        Self {
            header: Header {
                stamp: Time::new(0, 0),
                frame_id: String::new(),
            },
            name: Vec::new(),
            position: Vec::new(),
            velocity: Vec::new(),
            effort: Vec::new(),
        }
    }
}

pub struct CommunicationLayer {
    session: Session,
    joint_state_key: String,
    joint_command_key: String,
    image_key: String,
}

impl CommunicationLayer {
    pub async fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let config = zenoh::config::Config::default();
        let session = zenoh::open(config).await.map_err(|e| e.to_string())?;

        Ok(Self {
            session,
            joint_state_key: "rt/robot/joint_states".to_string(),
            joint_command_key: "rt/robot/joint_commands".to_string(),
            image_key: "rt/camera/image_raw".to_string(),
        })
    }

    pub async fn publish_joint_command(&self, joints: &[KinematicsJointState]) -> Result<(), Box<dyn std::error::Error>> {
        let msg = self.convert_to_ros_joint_state(joints);
        // Prepend ROS 2 CDR encapsulation header (Little Endian: 0x00 0x01 0x00 0x00)
        let mut payload = vec![0x00, 0x01, 0x00, 0x00];
        let data = cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite)?;
        payload.extend(data);
        self.session.put(&self.joint_command_key, payload).await.map_err(|e| e.to_string())?;
        Ok(())
    }

    pub async fn subscribe_joint_state<F>(&self, callback: F) -> Result<(), Box<dyn std::error::Error>>
    where F: Fn(Vec<KinematicsJointState>) + Send + Sync + 'static
    {
        let subscriber = self.session.declare_subscriber(&self.joint_state_key).await.map_err(|e| e.to_string())?;

        tokio::spawn(async move {
            while let Ok(sample) = subscriber.recv_async().await {
                 let payload = sample.payload().to_bytes();
                 // Check and skip ROS 2 CDR encapsulation header (4 bytes)
                 if payload.len() > 4 {
                     // We assume Little Endian for simplicity
                     let mut deserializer = cdr::Deserializer::<_, _, cdr::LittleEndian>::new(&payload[4..], cdr::Infinite);
                     match serde::Deserialize::deserialize(&mut deserializer) {
                         Ok(msg) => {
                             let msg: JointState = msg;
                             let joints = Self::convert_from_ros_joint_state(&msg);
                             callback(joints);
                         }
                         Err(_) => eprintln!("Failed to deserialize JointState"),
                     }
                 } else {
                     eprintln!("Received payload too short for ROS 2 message");
                 }
            }
        });
        Ok(())
    }

    pub async fn subscribe_camera_image<F>(&self, callback: F) -> Result<(), Box<dyn std::error::Error>>
    where F: Fn(Image) + Send + Sync + 'static
    {
        let subscriber = self.session.declare_subscriber(&self.image_key).await.map_err(|e| e.to_string())?;

        tokio::spawn(async move {
            while let Ok(sample) = subscriber.recv_async().await {
                let payload = sample.payload().to_bytes();
                if payload.len() > 4 {
                    let mut deserializer = cdr::Deserializer::<_, _, cdr::LittleEndian>::new(&payload[4..], cdr::Infinite);
                    match serde::Deserialize::deserialize(&mut deserializer) {
                        Ok(msg) => {
                            let msg: Image = msg;
                            callback(msg);
                        }
                        Err(_) => eprintln!("Failed to deserialize Image"),
                    }
                } else {
                    eprintln!("Received payload too short for ROS 2 message");
                }
            }
        });
        Ok(())
    }

    fn convert_to_ros_joint_state(&self, joints: &[KinematicsJointState]) -> JointState {
        let mut msg = JointState::default();

        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
        msg.header.stamp = Time::new(now.as_secs() as i32, now.subsec_nanos());
        msg.header.frame_id = "robot_base".to_string();

        for (i, joint) in joints.iter().enumerate() {
            msg.name.push(format!("joint_{}", i + 1));
            msg.position.push(joint.angle);
            msg.velocity.push(joint.velocity);
            msg.effort.push(joint.effort);
        }
        msg
    }

    fn convert_from_ros_joint_state(msg: &JointState) -> Vec<KinematicsJointState> {
        let mut joints = Vec::new();
        let len = msg.position.len();
        for i in 0..len {
            joints.push(KinematicsJointState {
                angle: msg.position[i],
                velocity: if i < msg.velocity.len() { msg.velocity[i] } else { 0.0 },
                effort: if i < msg.effort.len() { msg.effort[i] } else { 0.0 },
            });
        }
        joints
    }
}
