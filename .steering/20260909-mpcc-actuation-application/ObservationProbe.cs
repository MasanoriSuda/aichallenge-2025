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
      Console.WriteLine("MPCC_INPUT_OBS apply id="+RuntimeHelpers.GetHashCode(receiver)+
        " seq="+s.SelectedSequence+" source_ns="+s.SelectedSourceNs+" ros_ns="+Ns(rosTime)+
        " input="+Text(input)+" unity="+Text(unityTime)+" selected_input="+Text(s.SelectedInput)+
        " selected_unity="+Text(s.SelectedUnityTime)+" emergency="+emergency+
        " mono_ticks="+Stopwatch.GetTimestamp()+" topic="+Member(receiver,"ackermannControlCommandTopic"));
    } catch(Exception e) {Console.WriteLine("MPCC_INPUT_OBS error apply "+e.GetType().Name);}
  }
}
