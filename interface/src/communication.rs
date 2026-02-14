use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::sensor_msgs::Image;
use edgefirst_schemas::std_msgs::Header;
use kinematics::JointState as KinematicsJointState;
use serde::{Deserialize, Serialize};
use std::time::{SystemTime, UNIX_EPOCH};
use zenoh::Session;

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

const ROS2_CDR_HEADER_LE: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

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

    pub async fn publish_joint_command(
        &self,
        joints: &[KinematicsJointState],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let msg = self.convert_to_ros_joint_state(joints);
        // Prepend ROS 2 CDR encapsulation header (Little Endian: 0x00 0x01 0x00 0x00)
        let mut payload = ROS2_CDR_HEADER_LE.to_vec();
        let data = cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite)?;
        payload.extend(data);
        self.session
            .put(&self.joint_command_key, payload)
            .await
            .map_err(|e| e.to_string())?;
        Ok(())
    }

    pub async fn subscribe_joint_state<F>(
        &self,
        callback: F,
    ) -> Result<(), Box<dyn std::error::Error>>
    where
        F: Fn(Vec<KinematicsJointState>) + Send + Sync + 'static,
    {
        let subscriber = self
            .session
            .declare_subscriber(&self.joint_state_key)
            .await
            .map_err(|e| e.to_string())?;

        tokio::spawn(async move {
            while let Ok(sample) = subscriber.recv_async().await {
                let payload = sample.payload().to_bytes();
                match Self::deserialize_ros_payload::<JointState>(&payload) {
                    Ok(msg) => {
                        let joints = Self::convert_from_ros_joint_state(&msg);
                        callback(joints);
                    }
                    Err(e) => eprintln!("Failed to process JointState: {}", e),
                }
            }
        });
        Ok(())
    }

    pub async fn subscribe_camera_image<F>(
        &self,
        callback: F,
    ) -> Result<(), Box<dyn std::error::Error>>
    where
        F: Fn(Image) + Send + Sync + 'static,
    {
        let subscriber = self
            .session
            .declare_subscriber(&self.image_key)
            .await
            .map_err(|e| e.to_string())?;

        tokio::spawn(async move {
            while let Ok(sample) = subscriber.recv_async().await {
                let payload = sample.payload().to_bytes();
                match Self::deserialize_ros_payload::<Image>(&payload) {
                    Ok(msg) => {
                        callback(msg);
                    }
                    Err(e) => eprintln!("Failed to process Image: {}", e),
                }
            }
        });
        Ok(())
    }

    fn deserialize_ros_payload<T>(payload: &[u8]) -> Result<T, Box<dyn std::error::Error>>
    where
        T: for<'de> serde::Deserialize<'de>,
    {
        if payload.len() < 4 {
            return Err("Payload too short for ROS 2 CDR header".into());
        }
        if payload[0..4] != ROS2_CDR_HEADER_LE {
            return Err("Invalid ROS 2 CDR header".into());
        }
        let mut deserializer =
            cdr::Deserializer::<_, _, cdr::LittleEndian>::new(&payload[4..], cdr::Infinite);
        let msg = serde::Deserialize::deserialize(&mut deserializer)?;
        Ok(msg)
    }

    fn convert_to_ros_joint_state(&self, joints: &[KinematicsJointState]) -> JointState {
        let mut msg = JointState::default();

        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default();
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
                velocity: if i < msg.velocity.len() {
                    msg.velocity[i]
                } else {
                    0.0
                },
                effort: if i < msg.effort.len() {
                    msg.effort[i]
                } else {
                    0.0
                },
            });
        }
        joints
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deserialize_ros_payload_valid() {
        let msg = JointState::default();
        let mut payload = ROS2_CDR_HEADER_LE.to_vec();
        let data = cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).unwrap();
        payload.extend(data);

        let result: Result<JointState, _> = CommunicationLayer::deserialize_ros_payload(&payload);
        assert!(result.is_ok());
    }

    #[test]
    fn test_deserialize_ros_payload_invalid_header() {
        let payload = vec![0x00, 0x00, 0x00, 0x00, 0x01, 0x02];
        let result: Result<JointState, _> = CommunicationLayer::deserialize_ros_payload(&payload);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err().to_string(), "Invalid ROS 2 CDR header");
    }

    #[test]
    fn test_deserialize_ros_payload_too_short() {
        let payload = vec![0x00, 0x01, 0x00];
        let result: Result<JointState, _> = CommunicationLayer::deserialize_ros_payload(&payload);
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err().to_string(),
            "Payload too short for ROS 2 CDR header"
        );
    }
}
