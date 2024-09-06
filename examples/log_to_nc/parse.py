from bst_python_sdk.comm_packets.comm_packets import ThreeAxisSensor
from bst_python_sdk.logparse import parse_log
from bst_python_sdk.handler import standard_handler

from enum import Enum

from netCDF4 import Dataset
import os.path
import sys

type_conv = {int: 'i8', float: 'f8'}


def parse(filename):
	parsed_log = parse_log(filename, standard_handler, True)
	convert(filename, parsed_log)


def read_var(pkt, var_name):
	if '.' not in var_name:
		return getattr(pkt, var_name)

	var_attrs = var_name.split('.')
	top_attr = var_attrs[0]
	sub_attr = var_attrs[1]

	return getattr(getattr(pkt, top_attr), sub_attr)


def convert(filename, parsed_log):
	nc_name = '.'.join(filename.split('.')[:-1]) + '.nc'
	root_grp = Dataset(nc_name, 'w', format='NETCDF4')

	for pkt_type, pkts in parsed_log.items():
		print(f'Adding {pkt_type.name}...')
		if len(pkts) == 0:
			print(' -- Skipping (dimension of size 0)')
			continue

		pkt_grp = root_grp.createGroup(pkt_type.name)
		pkt_grp.createDimension('packets', len(pkts))

		if type(pkts[0]) == int:
			continue

		for field in pkts[0].__dict__:
			def _parse_group(field):
				if field == 'buffer':
					return

				field_val = read_var(pkts[0], field)
				field_type = type(field_val)
				if isinstance(field_val, Enum):
					add_enum_to_nc(field, pkt_grp, pkts)
				elif field_type == list and type(field_val[0]) in type_conv:
					add_list_to_nc(field, field_val, pkt_grp, pkts)
				elif field_type in type_conv:
					add_primitive_to_nc(field, field_type, pkt_grp, pkts)
				elif field_type == ThreeAxisSensor:
					for sub_field in field_val.__dict__:
						field_name = f'{field}.{sub_field}'
						_parse_group(field_name)
				else:
					if field_type == list:
						print(f'Unsupported list type: {type(field_val)}')
					else:
						print(f'Unsupported type: {field_type}')

			_parse_group(field)


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


if __name__ == '__main__':
	if len(sys.argv) < 2:
		print('Not enough arguments: python parse.py <log name>')
		sys.exit(1)

	log_name = sys.argv[1]
	if not os.path.isfile(log_name):
		print(f'File "{log_name}" not found')
		sys.exit(1)

	parse(log_name)
