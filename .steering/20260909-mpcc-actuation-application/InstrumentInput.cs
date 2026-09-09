// Adds calls only to a generated copy. The installed simulator DLL is read-only.
using System;
using System.IO;
using System.Linq;
using Mono.Cecil;
using Mono.Cecil.Cil;
public static class InstrumentInput {
  private static void Append(ILProcessor il,Instruction anchor,params Instruction[] items) {
    foreach(var item in items){il.InsertAfter(anchor,item);anchor=item;}
  }
  private static void LongBranches(MethodDefinition method) {
    var all=typeof(OpCodes).GetFields().Where(f=>f.FieldType==typeof(OpCode)).Select(f=>(OpCode)f.GetValue(null)).ToArray();
    foreach(var i in method.Body.Instructions)
      if(i.OpCode.OperandType==OperandType.ShortInlineBrTarget)
        i.OpCode=all.Single(o=>o.Name==i.OpCode.Name.Substring(0,i.OpCode.Name.Length-2));
  }
  public static void Main(string[] args) {
    if(args.Length!=3)throw new ArgumentException("original.dll probe.dll output.dll");
    var resolver=new DefaultAssemblyResolver();resolver.AddSearchDirectory(Path.GetDirectoryName(args[0]));
    var module=ModuleDefinition.ReadModule(args[0],new ReaderParameters{AssemblyResolver=resolver});
    var probe=ModuleDefinition.ReadModule(args[1]);var pt=probe.Types.Single(t=>t.Name=="MPCCInputObservation");
    Func<string,MethodReference> method=n=>module.ImportReference(pt.Methods.Single(m=>m.Name==n));
    var type=module.Types.Single(t=>t.Name=="VehicleRosInput");
    var receive=type.Methods.Single(m=>m.Name=="<Start>b__53_2");
    var chosen=type.Methods.Single(m=>m.Name=="UpdateQueuedLongitudinalInput");
    var latest=type.Fields.Single(f=>f.Name=="latestLongitudinalInput");
    var vehicle=type.Fields.Single(f=>f.Name=="vehicle");
    var acceleration=module.Types.Single(t=>t.Name=="Vehicle").Fields.Single(f=>f.Name=="AccelerationInput");
    var ros=module.Types.Single(t=>t.Name=="SimulatorROS2Node").Methods.Single(m=>m.Name=="GetCurrentRosTime");
    var rx=receive.Body.Instructions.Single(i=>i.OpCode==OpCodes.Stfld && i.Operand==latest);
    Append(receive.Body.GetILProcessor(),rx,Instruction.Create(OpCodes.Ldarg_0),Instruction.Create(OpCodes.Ldarg_1),
      Instruction.Create(OpCodes.Ldarg_0),Instruction.Create(OpCodes.Ldfld,latest),Instruction.Create(OpCodes.Call,method("Receive")));
    var take=chosen.Body.Instructions.Single(i=>i.OpCode==OpCodes.Ldfld && i.Operand==latest).Next;
    if(take.OpCode!=OpCodes.Stloc_1)throw new Exception("unexpected receiver local layout");
    Append(chosen.Body.GetILProcessor(),take,Instruction.Create(OpCodes.Ldarg_0),Instruction.Create(OpCodes.Ldloc_1),
      Instruction.Create(OpCodes.Ldloc_3),Instruction.Create(OpCodes.Call,method("Choose")));
    var writes=chosen.Body.Instructions.Where(i=>i.OpCode==OpCodes.Stfld && i.Operand==acceleration).ToArray();
    if(writes.Length!=2)throw new Exception("expected emergency and regular application");
    for(int j=0;j<writes.Length;++j) {
      var instructions=new System.Collections.Generic.List<Instruction>{Instruction.Create(OpCodes.Ldarg_0),
        Instruction.Create(OpCodes.Ldarg_0),Instruction.Create(OpCodes.Ldfld,vehicle),Instruction.Create(OpCodes.Ldfld,acceleration),
        Instruction.Create(OpCodes.Ldloc_3),Instruction.Create(OpCodes.Call,ros)};
      if(ros.ReturnType.IsValueType)instructions.Add(Instruction.Create(OpCodes.Box,ros.ReturnType));
      instructions.Add(Instruction.Create(j==0?OpCodes.Ldc_I4_1:OpCodes.Ldc_I4_0));
      instructions.Add(Instruction.Create(OpCodes.Call,method("Apply")));
      Append(chosen.Body.GetILProcessor(),writes[j],instructions.ToArray());
    }
    LongBranches(receive);LongBranches(chosen);
    module.Write(args[2]);
    Console.WriteLine("Added Receive1/Choose1/Apply2 probes; original application data and branch targets preserved.");
    Console.WriteLine("ROS time return type: "+ros.ReturnType.FullName);
  }
}
