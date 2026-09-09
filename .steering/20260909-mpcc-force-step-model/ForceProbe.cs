// Read-only diagnostic oracle. Never linked into the participant controller.
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Text;
using UnityEngine;

public static class MPCCForceObservation {
  private sealed class WheelState {
    public object Value;
    public WheelCollider Collider;
    public bool Called;
    public Vector3 LateralForce, DriveAcceleration;
  }
  private sealed class State {
    public Rigidbody Body;
    public List<WheelState> Wheels=new List<WheelState>();
    public Vector3 P,V,W,F,T;
    public Quaternion Q;
    public float Wire;
    public bool Active;
    public long Tick;
  }
  private static readonly ConditionalWeakTable<object,State> states=new ConditionalWeakTable<object,State>();
  private static readonly Dictionary<object,WheelState> wheels=new Dictionary<object,WheelState>();
  private static object Get(object o,string name) {
    const BindingFlags flags=BindingFlags.Instance|BindingFlags.Public|BindingFlags.NonPublic;
    var type=o.GetType();var field=type.GetField(name,flags);
    if(field!=null)return field.GetValue(o);
    var property=type.GetProperty(name,flags);
    if(property!=null)return property.GetValue(o,null);
    throw new MissingMemberException(type.FullName,name);
  }
  private static State Create(object vehicle) {
    var state=new State();state.Body=(Rigidbody)Get(vehicle,"m_rigidbody");
    foreach(var value in (IEnumerable)Get(vehicle,"wheels")) {
      var wheel=new WheelState{Value=value,Collider=(WheelCollider)Get(value,"wheelCollider")};
      state.Wheels.Add(wheel);wheels.Add(value,wheel);
    }
    if(state.Wheels.Count!=4)throw new Exception("expected four wheels");
    return state;
  }
  private static void Number(StringBuilder text,string key,double value) {
    text.Append(' ').Append(key).Append('=').Append(value.ToString("R",CultureInfo.InvariantCulture));
  }
  private static void Vec(StringBuilder text,string key,Vector3 v) {
    Number(text,key+"x",v.x);Number(text,key+"y",v.y);Number(text,key+"z",v.z);
  }
  private static void Quat(StringBuilder text,string key,Quaternion q) {
    Number(text,key+"x",q.x);Number(text,key+"y",q.y);Number(text,key+"z",q.z);Number(text,key+"w",q.w);
  }
  public static void Begin(object vehicle) {
    try {
      var state=states.GetValue(vehicle,Create);var body=state.Body;
      if(state.Active)throw new Exception("nested physics begin");
      state.Active=true;state.Tick++;
      state.P=body.position;state.Q=body.rotation;state.V=body.velocity;state.W=body.angularVelocity;
      state.F=body.GetAccumulatedForce(Time.fixedDeltaTime);state.T=body.GetAccumulatedTorque(Time.fixedDeltaTime);
      state.Wire=Convert.ToSingle(Get(vehicle,"AccelerationInput"));
      foreach(var wheel in state.Wheels){wheel.Called=false;wheel.LateralForce=Vector3.zero;wheel.DriveAcceleration=Vector3.zero;}
    }catch(Exception error){Console.WriteLine("MPCC_FORCE_ERROR begin "+error);}
  }
  // Values are the actual CIL locals supplied to AddForceAtPosition.
  public static void Wheel(object value,Vector3 lateralForce,Vector3 driveAcceleration) {
    try {
      var state=wheels[value];
      if(state.Called)throw new Exception("duplicate wheel force within step");
      state.Called=true;state.LateralForce=lateralForce;state.DriveAcceleration=driveAcceleration;
    }catch(Exception error){Console.WriteLine("MPCC_FORCE_ERROR wheel "+error);}
  }
  public static void End(object vehicle) {
    try {
      var state=states.GetValue(vehicle,Create);var body=state.Body;
      if(!state.Active)throw new Exception("physics end without begin");
      var text=new StringBuilder(4000);text.Append("MPCC_FORCE_OBS id=").Append(RuntimeHelpers.GetHashCode(vehicle));
      Number(text,"tick",state.Tick);Number(text,"fixed",Time.fixedTimeAsDouble);Number(text,"dt",Time.fixedDeltaTime);
      Number(text,"wire_before",state.Wire);Number(text,"wire",Convert.ToSingle(Get(vehicle,"AccelerationInput")));
      Number(text,"gear",Convert.ToInt32(Get(vehicle,"AutomaticShiftInput")));
      Number(text,"sleep",Convert.ToBoolean(Get(vehicle,"lastSleep"))?1:0);
      Number(text,"vehicle_speed",Convert.ToSingle(Get(vehicle,"Speed")));
      Number(text,"steer_input_deg",Convert.ToSingle(Get(vehicle,"SteerAngleInput")));
      Number(text,"steer_deg",Convert.ToSingle(Get(vehicle,"actualSteerAngle")));
      Number(text,"mass",body.mass);Number(text,"drag",body.drag);Number(text,"angular_drag",body.angularDrag);
      Vec(text,"bp_",state.P);Quat(text,"bq_",state.Q);Vec(text,"bv_",state.V);Vec(text,"bw_",state.W);
      Vec(text,"ep_",body.position);Quat(text,"eq_",body.rotation);Vec(text,"ev_",body.velocity);Vec(text,"ew_",body.angularVelocity);
      Vec(text,"bf_",state.F);Vec(text,"bt_",state.T);
      Vec(text,"ef_",body.GetAccumulatedForce(Time.fixedDeltaTime));Vec(text,"et_",body.GetAccumulatedTorque(Time.fixedDeltaTime));
      Vec(text,"com_",body.worldCenterOfMass);Vec(text,"inertia_",body.inertiaTensor);Quat(text,"iq_",body.inertiaTensorRotation);
      Vec(text,"gravity_",Physics.gravity);Number(text,"use_gravity",body.useGravity?1:0);
      for(int i=0;i<state.Wheels.Count;++i) {
        var wheel=state.Wheels[i];string key="w"+i+"_";
        var hit=(WheelHit)Get(wheel.Value,"wheelHit");
        Number(text,key+"ground",Convert.ToBoolean(Get(wheel.Value,"IsGrounded"))?1:0);
        Number(text,key+"called",wheel.Called?1:0);Number(text,key+"sprung",wheel.Collider.sprungMass);
        Number(text,key+"steer",wheel.Collider.steerAngle);
        Vec(text,key+"p_",hit.point);Vec(text,key+"n_",hit.normal);Vec(text,key+"f_",hit.forwardDir);Vec(text,key+"s_",hit.sidewaysDir);
        Vec(text,key+"v_",body.GetPointVelocity(hit.point));Vec(text,key+"lat_",wheel.LateralForce);Vec(text,key+"drive_",wheel.DriveAcceleration);
      }
      Console.WriteLine(text.ToString());state.Active=false;
    }catch(Exception error){Console.WriteLine("MPCC_FORCE_ERROR end "+error);}
  }
}
