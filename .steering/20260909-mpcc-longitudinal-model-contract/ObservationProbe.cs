// Diagnostic only. Never used by participant controller or submission.
using System;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Diagnostics;
using System.Globalization;
public static class MPCCInputObservation {
  private sealed class State {
    public long Sequence, SourceNs, SelectedSequence, SelectedSourceNs;
    public float SelectedInput, SelectedUnityTime;
    public bool PhysicsLogged;
  }
  private static readonly ConditionalWeakTable<object, State> states = new ConditionalWeakTable<object, State>();
  private static object Member(object o, string name) {
    if (o == null) return null;
    const BindingFlags f = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.IgnoreCase;
    var p=o.GetType().GetProperty(name,f); if(p!=null)return p.GetValue(o,null);
    var v=o.GetType().GetField(name,f);return v==null?null:v.GetValue(o);
  }
  private static long Ns(object stamp) {
    var sec=Member(stamp,"Sec");var nano=Member(stamp,"Nanosec");
    return sec==null || nano==null ? -1 : checked(Convert.ToInt64(sec)*1000000000L+Convert.ToInt64(nano));
  }
  private static string Text(float x) {return x.ToString("R",CultureInfo.InvariantCulture);}
  private static object Required(object o,string name) {
    var value=Member(o,name);if(value==null)throw new MissingMemberException(o==null?"null":o.GetType().FullName,name);return value;
  }
  private static string Scalar(object o,string name) {
    var value=Required(o,name);return value is float?Text((float)value):Convert.ToString(value,CultureInfo.InvariantCulture);
  }
  private static string Vector(string label,object value) {
    return " "+label+"x="+Scalar(value,"x")+" "+label+"y="+Scalar(value,"y")+" "+label+"z="+Scalar(value,"z");
  }
  private static void Physics(object receiver,State state) {
    var vehicle=Required(receiver,"vehicle");
    if(state.PhysicsLogged || Convert.ToSingle(Required(vehicle,"Speed"))<=.5f)return;
    var body=Required(vehicle,"m_rigidbody");var transform=Required(body,"transform");
    string prefix="MPCC_PHYSICS_OBS id="+RuntimeHelpers.GetHashCode(receiver)+" source_ns="+state.SelectedSourceNs;
    Console.WriteLine(prefix+" mass="+Scalar(body,"mass")+" drag="+Scalar(body,"drag")+" angular_drag="+Scalar(body,"angularDrag")+
      " skid="+Scalar(vehicle,"SkiddingCancelRate")+" rolling="+Scalar(vehicle,"rollingResistance")+
      " max_a="+Scalar(vehicle,"MaxAccelerationInput")+" max_d="+Scalar(vehicle,"MaxDecelerationInput")+
      " grip="+Scalar(vehicle,"gripSteerFactor")+" steer_delay="+Scalar(vehicle,"steerDelayTime")+
      " steer_tau="+Scalar(vehicle,"steerTimeConstant")+" sleep_v="+Scalar(vehicle,"sleepVelocityThreshold")+
      " sleep_time="+Scalar(vehicle,"sleepTimeThreshold")+
      Vector("body_",Required(transform,"position"))+Vector("com_local_",Required(body,"centerOfMass"))+
      Vector("com_world_",Required(body,"worldCenterOfMass"))+Vector("inertia_",Required(body,"inertiaTensor"))+
      Vector("rotation_",Required(transform,"rotation"))+" rotation_w="+Scalar(Required(transform,"rotation"),"w"));
    int i=0;
    foreach(var wheel in (System.Collections.IEnumerable)Required(vehicle,"wheels")) {
      var collider=Required(wheel,"wheelCollider");
      Console.WriteLine(prefix+" wheel="+i+" sprung_mass="+Scalar(collider,"sprungMass")+" wheel_mass="+Scalar(collider,"mass")+
        " grounded="+Scalar(wheel,"IsGrounded")+" steer_deg="+Scalar(collider,"steerAngle")+
        " radius="+Scalar(collider,"radius")+" damping="+Scalar(collider,"wheelDampingRate")+
        Vector("position_",Required(Required(collider,"transform"),"position")));
      ++i;
    }
    state.PhysicsLogged=true;
  }
  public static void Receive(object receiver,object message,float input) {
    try {
      var s=states.GetOrCreateValue(receiver); ++s.Sequence;
      var stamp=Member(message,"Stamp") ?? Member(Member(message,"Header"),"Stamp");
      s.SourceNs=Ns(stamp);
      Console.WriteLine("MPCC_INPUT_OBS rx id="+RuntimeHelpers.GetHashCode(receiver)+" seq="+s.Sequence+
        " source_ns="+s.SourceNs+" input="+Text(input)+" mono_ticks="+Stopwatch.GetTimestamp()+
        " topic="+Member(receiver,"ackermannControlCommandTopic"));
    } catch(Exception e) {Console.WriteLine("MPCC_INPUT_OBS error rx "+e.GetType().Name);}
  }
  // Called while the original actuation lock is held, at its actual selection.
  public static void Choose(object receiver,float input,float unityTime) {
    try {
      var s=states.GetOrCreateValue(receiver);
      s.SelectedSequence=s.Sequence;s.SelectedSourceNs=s.SourceNs;
      s.SelectedInput=input;s.SelectedUnityTime=unityTime;
    } catch(Exception e) {Console.WriteLine("MPCC_INPUT_OBS error choose "+e.GetType().Name);}
  }
  // Called after the original Vehicle.AccelerationInput assignment and unlock.
  public static void Apply(object receiver,float input,float unityTime,object rosTime,bool emergency) {
    try {
      var s=states.GetOrCreateValue(receiver);
      Physics(receiver,s);
      Console.WriteLine("MPCC_INPUT_OBS apply id="+RuntimeHelpers.GetHashCode(receiver)+
        " seq="+s.SelectedSequence+" source_ns="+s.SelectedSourceNs+" ros_ns="+Ns(rosTime)+
        " input="+Text(input)+" unity="+Text(unityTime)+" selected_input="+Text(s.SelectedInput)+
        " selected_unity="+Text(s.SelectedUnityTime)+" emergency="+emergency+
        " mono_ticks="+Stopwatch.GetTimestamp()+" topic="+Member(receiver,"ackermannControlCommandTopic"));
    } catch(Exception e) {Console.WriteLine("MPCC_INPUT_OBS error apply "+e.GetType().Name);}
  }
}
