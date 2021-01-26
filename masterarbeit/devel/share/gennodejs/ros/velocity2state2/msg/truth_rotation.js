// Auto-generated. Do not edit!

// (in-package velocity2state2.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let tf = _finder('tf');

//-----------------------------------------------------------

class truth_rotation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.send_transform = null;
    }
    else {
      if (initObj.hasOwnProperty('send_transform')) {
        this.send_transform = initObj.send_transform
      }
      else {
        this.send_transform = new tf.msg.tfMessage();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type truth_rotation
    // Serialize message field [send_transform]
    bufferOffset = tf.msg.tfMessage.serialize(obj.send_transform, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type truth_rotation
    let len;
    let data = new truth_rotation(null);
    // Deserialize message field [send_transform]
    data.send_transform = tf.msg.tfMessage.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += tf.msg.tfMessage.getMessageSize(object.send_transform);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'velocity2state2/truth_rotation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '57c643213939b2acaf9cdeb59405057e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    tf/tfMessage send_transform
    
    ================================================================================
    MSG: tf/tfMessage
    geometry_msgs/TransformStamped[] transforms
    
    ================================================================================
    MSG: geometry_msgs/TransformStamped
    # This expresses a transform from coordinate frame header.frame_id
    # to the coordinate frame child_frame_id
    #
    # This message is mostly used by the 
    # <a href="http://wiki.ros.org/tf">tf</a> package. 
    # See its documentation for more information.
    
    Header header
    string child_frame_id # the frame id of the child frame
    Transform transform
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Transform
    # This represents the transform between two coordinate frames in free space.
    
    Vector3 translation
    Quaternion rotation
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new truth_rotation(null);
    if (msg.send_transform !== undefined) {
      resolved.send_transform = tf.msg.tfMessage.Resolve(msg.send_transform)
    }
    else {
      resolved.send_transform = new tf.msg.tfMessage()
    }

    return resolved;
    }
};

module.exports = truth_rotation;
