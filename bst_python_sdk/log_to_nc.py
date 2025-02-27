import sys
import os.path

from .logparse import Parser
from enum import Enum
import inspect

from netCDF4 import Dataset

type_conv = {int: 'i8', float: 'f8'}

script_dir = os.path.dirname(__file__)
root_dir = os.path.abspath(os.path.join(script_dir, 'bst_python_sdk'))
sys.path.insert(0, root_dir)


def convert_to_nc(
	filename: str,
	has_addr: bool=False,
	quick_mode: bool=False,
	out_dir: str='.',
) -> list[str]:
	while out_dir.endswith('/'):
		out_dir = out_dir[:len(out_dir)-1]
	parser = Parser(has_addr=has_addr, quick_mode=quick_mode)
	parsed_log = parser.parse_log(filename)
	converted = []
	for name in parsed_log.keys():
		if len(parsed_log[name].items()) == 0:
			continue
		nc_name = convert(filename, parsed_log[name], name, out_dir)
		converted.append(nc_name)

	return converted


def read_var(pkt, var_name):
	if '.' not in var_name:
		return getattr(pkt, var_name)

	var_attrs = var_name.split('.')
	result = getattr(pkt, var_attrs[0])
	for i in range(1, len(var_attrs)):
		result = getattr(result, var_attrs[i])

	return result


def convert(filename: str, parsed_log: dict, ac_name: str, out_dir: str) -> str:
	print(f'\n### Converting {ac_name}')
	log_name = '.'.join(filename.split('.')[:-1])
	nc_name = f'{log_name}_{ac_name}.nc'
	nc_name = f'{out_dir}/{nc_name.split("/")[-1]}'
	root_grp = Dataset(nc_name, 'w', format='NETCDF4')

	for pkt_type, pkts in parsed_log.items():
		print(f'Adding {pkt_type}...')
		if len(pkts) == 0:
			print(' -- Skipping (dimension of size 0)')
			continue

		pkt_grp = root_grp.createGroup(pkt_type)
		pkt_grp.createDimension('packets', len(pkts))

		if type(pkts[0]) == int:
			continue

		for field in pkts[0].__dict__:
			def _parse_field(field):
				field_val = read_var(pkts[0], field)
				field_type = type(field_val)
				if field == 'system_time':
					field_type = float
				if isinstance(field_val, Enum):
					add_enum_to_nc(field, pkt_grp, pkts)
				elif field_type == list and type(field_val[0]) in type_conv:
					add_list_to_nc(field, field_val, pkt_grp, pkts)
				elif field_type in type_conv:
					add_primitive_to_nc(field, field_type, pkt_grp, pkts)
				elif inspect.isclass(field_type):
					for sub_field in field_val.__dict__:
						field_name = f'{field}.{sub_field}'
						_parse_field(field_name)
				else:
					if field_type == list:
						print(f'Unsupported list type: {type(field_val)}')
					else:
						print(f'Unsupported type: {field_type}')

			_parse_field(field)
	return nc_name


def add_enum_to_nc(field, pkt_grp, pkts):
	nc_type = 'ubyte'
	group_var = pkt_grp.createVariable(field, nc_type, ('packets',))
	group_var[:] = [read_var(pkt, field).value for pkt in pkts]


def add_list_to_nc(field, field_val, pkt_grp, pkts):
	l_dim = f'{field}_length'
	pkt_grp.createDimension(l_dim, len(field_val))
	nc_type = type_conv[type(field_val[0])]
	group_var = pkt_grp.createVariable(field, nc_type, ('packets', l_dim))
	group_var[:] = [read_var(pkt, field) for pkt in pkts]


def add_primitive_to_nc(field, field_type, pkt_grp, pkts):
	nc_type = type_conv[field_type]
	group_var = pkt_grp.createVariable(field, nc_type, ('packets',))
	group_var[:] = [read_var(pkt, field) for pkt in pkts]

