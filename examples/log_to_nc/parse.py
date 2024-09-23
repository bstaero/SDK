#!/usr/bin/env python

from bst_python_sdk.logparse import Parser
from bst_python_sdk.handler import standard_handler

import argparse
from enum import Enum
import inspect

from netCDF4 import Dataset
import os.path
import sys

type_conv = {int: 'i8', float: 'f8'}


def parse(filename, use_swig: bool, has_addr: bool):
	parser = Parser(use_swig=use_swig, has_addr=has_addr)
	parsed_log = parser.parse_log(filename)
	for name in parsed_log.keys():
		convert(filename, parsed_log[name], name)


def read_var(pkt, var_name):
	if '.' not in var_name:
		return getattr(pkt, var_name)

	var_attrs = var_name.split('.')
	result = getattr(pkt, var_attrs[0])
	for i in range(1, len(var_attrs)):
		result = getattr(result, var_attrs[i])

	return result


def convert(filename, parsed_log, ac_name):
	print(f'\n### Converting #{ac_name} log')
	nc_name = '.'.join(filename.split('.')[:-1]) + f'_{ac_name}.nc'
	nc_name = nc_name.split('/')[-1]
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
			def _parse_field(field):
				field_val = read_var(pkts[0], field)
				field_type = type(field_val)
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
	parser = argparse.ArgumentParser(
		prog='./parse.py',
		description='Convert GCS and aircraft logs to netCDF format')
	parser.add_argument('filepath', help='Path to the log file')
	parser.add_argument('-a', '--addr', action='store_true', default=False,
					 help='Enables addressing')
	parser.add_argument('-s', '--swig', action='store_true', default=False,
					 help='Enables SWIG log processing (must compile first)')
	args = parser.parse_args()

	if not os.path.isfile(args.filepath):
		print(f'File "{args.filepath}" not found')
		sys.exit(1)

	parse(args.filepath, args.swig, args.addr)
