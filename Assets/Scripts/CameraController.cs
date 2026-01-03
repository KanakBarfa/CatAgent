using UnityEngine;

public class CameraController : MonoBehaviour
{
    public Rigidbody2D target;

    void LateUpdate()
    {
        Vector3 temp = transform.position;
        temp.x = target.transform.position.x;
        transform.position = temp;
    }
}