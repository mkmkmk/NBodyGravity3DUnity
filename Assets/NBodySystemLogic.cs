using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor.ShaderGraph;
using UnityEngine;

public class NBodySystemLogic : MonoBehaviour
{
    [SerializeField] GameObject BodyPrefab;


    //private BodyState[] system;
    private GameObject[] gameSystem;

    // private float ScaleDim = 1f;
    private const float Game2LogicTimeMul = 10f;

    private const float TimeStep = 0.05f;

    private float time;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        Debug.Log("NBodySystemLogic.Start()");

        var startupSystem =

           new[]
            {
                    new Body
                    {
                        Mass = 1.5f,
                        Position = new Vector3(0, 0, 0),
                        Velocity = new Vector3(0, -0.01f),
                    },

                    new Body
                    {
                        Mass = 5e-2f,
                        Position = new Vector3(-21, 0),
                        Velocity = new Vector3(0, .3f),
                    },

                    new Body
                     {
                        Mass = 2e-9f,
                        Position= new Vector3(-21, -3),
                        Velocity= new Vector3(0.10f, 0.30f),

                    },
                    new Body{
                        Mass = 7e-9f,
                        Position= new Vector3(10, 0),
                        Velocity= new Vector3(0, -0.4f),

                    },

                    new Body{
                        Mass = 7e-9f,
                        Position= new Vector3(-12, 0),
                        Velocity= new Vector3(0, -0.4f),

                    },
            };

        gameSystem = startupSystem.Select(point =>
        {
            var go = Instantiate(BodyPrefab, Logic2Game(point.Position), Quaternion.identity, transform);
            go.GetComponent<BodyObject>().Body = point;
            return go;
        }).ToArray();

        time = Time.time;
    }

    private Vector3 Logic2Game(Vector3 inp) 
        => new (inp.x, inp.z, inp.y);


    // Update is called once per frame
    void Update()
    {
        var system = gameSystem.Select(item => item.GetComponent<BodyObject>().Body).ToArray();

        while (time < Time.time * Game2LogicTimeMul)
        {
            system = Rk4Step(system, TimeStep);
            time += TimeStep;
        }

        for (int i = 0; i < gameSystem.Length; i++)
        {
            var next = system[i];
            var item = gameSystem[i];
            var body = item.GetComponent<BodyObject>().Body;
            body.Position = next.Position;
            body.Velocity = next.Velocity;
            item.transform.position = Logic2Game(body.Position);
        }

    }


    private Body[] Rk4Step(IEnumerable<Body> points, float h)
    {
        var system = points as Body[] ?? points.ToArray();

        var k1 = F(system);

        var sys2 = Step(system, system, k1, h * .5f);
        var k2 = F(sys2);

        var sys3 = Step(system, sys2, k2, h * .5f);
        var k3 = F(sys3);

        var sys4 = Step(system, sys3, k3, h);
        var k4 = F(sys4);

        var dv =
            new[] { k1, Mul(2, k2), Mul(2, k3), k4, }
                .Aggregate((ki, kii) => ki.Zip(kii, (e1, e2) => e1 + e2).ToArray())
                .Select(el => el * h / 6)
                .ToArray();

        var dr =
            new[] {
                    system.Select(el => 1 * el.Velocity),
                        sys2.Select(el => 2 * el.Velocity),
                        sys3.Select(el => 2 * el.Velocity),
                        sys4.Select(el => 1 * el.Velocity),
                }
                .Aggregate((ki, kii) => ki.Zip(kii, (e1, e2) => e1 + e2).ToArray())
                .Select(el => el * h / 6)
                .ToArray();


        return Step(system, dv, dr);

    }


    private static Vector3[] Mul(float d, IEnumerable<Vector3> inp) 
        => inp.Select(el => el * d).ToArray();


    private Vector3[] F(IEnumerable<Body> system)
    {
        var tb = system as Body[] ?? system.ToArray();
        return
            tb
                .Select(sys => AccComp(sys, tb))
                .ToArray();
    }


    private static Body[] Step(Body[] toMove, Body[] state, Vector3[] acc, float h)
        => Step(toMove, Mul(h, acc), state.Select(el => el.Velocity * h));


    private static Body[] Step(IEnumerable<Body> system, Vector3[] dv, IEnumerable<Vector3> dr)
    {
        return
             system
              .Zip(dv, (sys, dvOne) => new { sys, dvOne })
              .Zip(dr, (pop, drOne) => new { pop.sys, pop.dvOne, drOne })
              .Select(
                    zipped => new Body
                    {
                        Mass = zipped.sys.Mass,
                        Velocity = zipped.sys.Velocity + zipped.dvOne,
                        Position = zipped.sys.Position + zipped.drOne,
                    })
              .ToArray();

    }


    private const float GConst = 1f;


    private Vector3 AccComp(Body me, IEnumerable<Body> system)
    {
        var allOther = system.Where(el => el != me);

        Vector3 force =
            allOther.Select(
                other =>
                {
                    var r = me.Position - other.Position;
                    float len = r.magnitude;
                    len = Math.Max(0.5f, len);
                    var acc = -GConst * other.Mass * me.Mass * r / len / len / len;
                    return acc;
                })
                .Aggregate((v1, v2) => v1 + v2);
        return force / me.Mass;


    }


}

