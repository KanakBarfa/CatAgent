using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class Myscript : Agent
{
    public Rigidbody2D body;
    public Rigidbody2D leftArm;
    public Rigidbody2D rightArm;
    public Rigidbody2D leftLeg;
    public Rigidbody2D rightLeg;
    public SpringJoint2D leftArmJoint;
    public SpringJoint2D rightArmJoint;
    public SpringJoint2D leftLegJoint;
    public SpringJoint2D rightLegJoint;
    public Rigidbody2D rightArmBack;
    public Rigidbody2D leftLegBack;
    public Rigidbody2D rightLegBack;
    public Rigidbody2D leftArmBack;
    public SpringJoint2D leftArmJointBack;
    public SpringJoint2D rightArmJointBack;
    public SpringJoint2D leftLegJointBack;
    public SpringJoint2D rightLegJointBack;

    private float defaultLegDistance = 0.66f;
    private float defaultArmDistance = 0.8f;
    public float endPenalty = -8f;
    public float bendPenalty = -2f;
    public float timeReward = 0.1f;
    public float velocityReward = 0.1f;

    private Vector2 leftArmInitialPos;
    private Vector2 rightArmInitialPos;
    private Vector2 leftLegInitialPos;
    private Vector2 rightLegInitialPos;

    // Save initial rotations
    private Quaternion leftArmInitialRot;
    private Quaternion rightArmInitialRot;
    private Quaternion leftLegInitialRot;
    private Quaternion rightLegInitialRot;

    public void Start()
    {
        // Save initial positions of arms and legs
        leftArmInitialPos = leftArm.transform.localPosition;
        rightArmInitialPos = rightArm.transform.localPosition;
        leftLegInitialPos = leftLeg.transform.localPosition;
        rightLegInitialPos = rightLeg.transform.localPosition;
        // Save initial rotations of arms and legs
        leftArmInitialRot = leftArm.transform.rotation;
        rightArmInitialRot = rightArm.transform.rotation;
        leftLegInitialRot = leftLeg.transform.rotation;
        rightLegInitialRot = rightLeg.transform.rotation;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(body.transform.localPosition.y);
        sensor.AddObservation(body.rotation);
        sensor.AddObservation(leftArm.rotation);
        sensor.AddObservation(leftArmBack.rotation);
        sensor.AddObservation(rightArm.rotation);
        sensor.AddObservation(rightArmBack.rotation);
        sensor.AddObservation(leftLeg.rotation);
        sensor.AddObservation(leftLegBack.rotation);
        sensor.AddObservation(rightLeg.rotation);
        sensor.AddObservation(rightLegBack.rotation);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var act = actions.ContinuousActions;

        // Set target angles for limbs
        leftArmJoint.distance = act[0]*defaultArmDistance*2;
        rightArmJoint.distance = act[1]*defaultArmDistance*2;
        leftLegJoint.distance = act[2]*defaultLegDistance*2;
        rightLegJoint.distance = act[3]*defaultLegDistance*2;
        leftArmJointBack.distance = act[4]*defaultArmDistance*2;
        rightArmJointBack.distance = act[5]*defaultArmDistance*2;
        leftLegJointBack.distance = act[6]*defaultLegDistance*2;
        rightLegJointBack.distance = act[7]*defaultLegDistance*2;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var act = actionsOut.ContinuousActions;
        // Example heuristic: random joint distances
        act[0] = Random.Range(0,defaultArmDistance);
        act[1] = Random.Range(0,defaultArmDistance);
        act[2] = Random.Range(0,defaultLegDistance);
        act[3] = Random.Range(0,defaultLegDistance);
        act[4] = Random.Range(0,defaultArmDistance);
        act[5] = Random.Range(0,defaultArmDistance);
        act[6] = Random.Range(0,defaultLegDistance);
        act[7] = Random.Range(0,defaultLegDistance);
    }

    public override void OnEpisodeBegin()
    {
        // Reset body position and velocity
        body.transform.localPosition = new Vector2(0f, -0.337f);
        //Rotate body by 90 degrees in Z axis
        body.transform.rotation = Quaternion.Euler(0f, 0f, -90f);
        body.linearVelocity = Vector2.zero;
        body.angularVelocity = 0f;

        // Reset limbs position and velocity
        leftArm.transform.localPosition = leftArmInitialPos;
        leftArm.transform.rotation = leftArmInitialRot;
        leftArm.linearVelocity = Vector2.zero;
        leftArm.angularVelocity = 0f;

        rightArm.transform.localPosition = rightArmInitialPos;
        rightArm.transform.rotation = rightArmInitialRot;
        rightArm.linearVelocity = Vector2.zero;
        rightArm.angularVelocity = 0f;

        leftLeg.transform.localPosition = leftLegInitialPos;
        leftLeg.transform.rotation = leftLegInitialRot;
        leftLeg.linearVelocity = Vector2.zero;
        leftLeg.angularVelocity = 0f;

        rightLeg.transform.localPosition = rightLegInitialPos;
        rightLeg.transform.rotation = rightLegInitialRot;
        rightLeg.linearVelocity = Vector2.zero;
        rightLeg.angularVelocity = 0f;

        leftArmBack.transform.localPosition = leftArmInitialPos;
        leftArmBack.transform.rotation = leftArmInitialRot;
        leftArmBack.linearVelocity = Vector2.zero;
        leftArmBack.angularVelocity = 0f;

        rightArmBack.transform.localPosition = rightArmInitialPos;
        rightArmBack.transform.rotation = rightArmInitialRot;
        rightArmBack.linearVelocity = Vector2.zero;
        rightArmBack.angularVelocity = 0f;

        leftLegBack.transform.localPosition = leftLegInitialPos;
        leftLegBack.transform.rotation = leftLegInitialRot;
        leftLegBack.linearVelocity = Vector2.zero;
        leftLegBack.angularVelocity = 0f;

        rightLegBack.transform.localPosition = rightLegInitialPos;
        rightLegBack.transform.rotation = rightLegInitialRot;
        rightLegBack.linearVelocity = Vector2.zero;
        rightLegBack.angularVelocity = 0f;

    }

    public void FixedUpdate()
    {
        AddReward(timeReward);
        AddReward(velocityReward * body.linearVelocity.x);
    }

    private void LateUpdate()
    {
        CheckShinFlip(leftLeg, leftArm);
        CheckShinFlip(rightArm, rightLeg);
        CheckShinFlip(leftLegBack, leftArmBack);
        CheckShinFlip(rightArmBack, rightLegBack);
    }

    private void CheckShinFlip(Rigidbody2D upper, Rigidbody2D lower)
    {
        // Calculate the angle between the "Up" vector of the upper leg and the lower leg.
        // If your sprites are drawn horizontally, change .up to .right
        float angle = Vector2.SignedAngle(upper.transform.up, lower.transform.up);

        if (angle < 0f) 
        {
            AddReward(bendPenalty); // Apply the penalty
            EndEpisode();          // Reset the agent
        }
    }

    // If touching ground, end episode
    private void OnCollisionEnter2D(Collision2D collision)
    {
        if (collision.gameObject.CompareTag("Wall"))
        {
            AddReward(endPenalty);
            EndEpisode();
        }
    }
}