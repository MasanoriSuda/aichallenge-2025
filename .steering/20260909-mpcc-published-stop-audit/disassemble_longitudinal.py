"""Read the local sensor producer and coordinate-conversion IL without executing it."""
from pathlib import Path
import dnfile
from dncil.cil.body.reader import read_method_body_from_bytes

path = Path('aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll')
pe = dnfile.dnPE(str(path))
owners = {id(m.row): str(t.TypeName) for t in pe.net.mdtables.TypeDef for m in t.MethodList}
fields = {id(f.row): str(t.TypeName) for t in pe.net.mdtables.TypeDef for f in t.FieldList}

def resolve(operand):
    if not hasattr(operand, 'value'):
        return str(operand)
    table_id, index = operand.value >> 24, operand.value & 0xffffff
    if table_id == 0x70:
        return repr(pe.net.user_strings.get(index).value)
    table = pe.net.mdtables.tables.get(table_id)
    if table and index:
        row = table.rows[index - 1]
        return owners.get(id(row), fields.get(id(row), '')) + '::' + str(
            getattr(row, 'Name', getattr(row, 'TypeName', operand)))
    return str(operand)

selected = {
    'VehicleRosInput': {'TryGetAckermannSteerInput','TryGetActuationSteerInput','Update'},
    'Vehicle': {'get_SteerAngle','<FixedUpdate>g__UpdateSteeringResponse|70_1','<FixedUpdate>g__GetDelayedSteerAngle|70_2','<FixedUpdate>g__ApplySteerRateLimit|70_3','<FixedUpdate>g__CalculateLimitedSteerAngle|70_4'},
    'Wheel': {'get_SteerAngle','UpdateWheelSteerAngle'},
    'VehicleReportRos2Publisher': {'FixedUpdate'},
}
lines = []
for type_row in pe.net.mdtables.TypeDef:
    for method in type_row.MethodList:
        if str(type_row.TypeName) != 'VehicleRosInput':
            continue
        lines.append(f'\n{type_row.TypeName}::{method.row.Name} RVA={method.row.Rva:x}')
        for instruction in read_method_body_from_bytes(pe.get_data(method.row.Rva)).instructions:
            lines.append(f'{instruction.offset:04x} {instruction.mnemonic:15s} {resolve(instruction.operand)}')
Path('output/20260909-published-stop-observation-dev2-r1/unity-longitudinal-cil.txt').write_text(
    '\n'.join(lines) + '\n')
