import tf

def tf_to_mat(trans, rot):
  # pose_mat : [4x4] array
  trans_mat = tf.transformations.translation_matrix(trans)
  rot_mat = tf.transformations.quaternion_matrix(rot)
  pose_mat = tf.transformations.concatenate_matrices(trans_mat, rot_mat)
  return pose_mat

def mat_to_tf(pose_mat):
  # pose_mat : [4x4] array
  trans = tf.transformations.translation_from_matrix(pose_mat)
  rot = tf.transformations.quaternion_from_matrix(pose_mat)
  return trans, rot
