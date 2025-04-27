using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddRigidbodyToObjects : MonoBehaviour
{
    public string[] tagPrefixes = { "Building_A1_prefab", "Building_I_2_Prefab", "rus_build_9et_01", "Regroup14" };

    void Start()
    {
        Debug.Log("Añadiendo Rigidbody y BoxCollider a los objetos...");

        // Obtener todos los objetos en la escena de forma más eficiente
        Transform[] allObjects = FindObjectsOfType<Transform>(); 

        foreach (Transform obj in allObjects)
        {
            // Recorrer cada prefijo para verificar si coincide con el nombre del objeto
            foreach (string tagPrefix in tagPrefixes)
            {
                if (obj.name.StartsWith(tagPrefix))
                {
                    // Verificar si el objeto ya tiene un Rigidbody
                    Rigidbody rb = obj.GetComponent<Rigidbody>();
                    if (rb == null)
                    {
                        rb = obj.gameObject.AddComponent<Rigidbody>();
                        Debug.Log("Rigidbody añadido a: " + obj.name);
                    }

                    // Configurar propiedades del Rigidbody
                    rb.mass = 10000f;  // 10 toneladas
                    rb.useGravity = true; // Desactivar la gravedad
                    rb.constraints = RigidbodyConstraints.FreezeRotation; // Evitar que los edificios se vuelquen

                    // Verificar si el objeto ya tiene un Collider
                    Collider col = obj.GetComponent<Collider>();
                    if (col == null)
                    {
                        BoxCollider boxCollider = obj.gameObject.AddComponent<BoxCollider>();
                        Debug.Log("BoxCollider añadido a: " + obj.name);
                    }

                    break; // Salir del loop de prefijos para este objeto
                }
            }
        }
    }
}

