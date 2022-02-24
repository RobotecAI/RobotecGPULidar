using System;
using System.Numerics;

namespace GPULidarRaycaster
{

public struct LidarSource {
  public String source_id;
  public float[] lidarPose;
  public float[] sourcePoses;
  public int[] lidarArrayRingIds;
  public float[] postRaycastTransform;
  public float range;
}

} // namespace GPULidarRaycaster
