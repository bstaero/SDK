from lxml import etree

type_conv = {
    "CHAR": {
        "default": 0,
        "unpack": "<c"
    },
    "INT8": {
        "default": 0,
        "unpack": "<b"
    },
    "UINT8": {
        "default": 0,
        "unpack": "<B"
    },
    "INT16": {
        "default": 0,
        "unpack": "<h"
    },
    "UINT16": {
        "default": 0,
        "unpack": "<H"
    },
    "INT32": {
        "default": 0,
        "unpack": "<i"
    },
    "UINT32": {
        "default": 0,
        "unpack": "<I"
    },
    "FLOAT": {
        "default": 0.0,
        "unpack": "<f",
    },
    "INT64": {
        "default": 0,
        "unpack": "<d",
    },
    "UINT64": {
        "default": 0,
        "unpack": "<Q",
    },
}

class XMLUserPayload:
    def __init__(self, xml_path):
        root = etree.parse(xml_path).getroot()
        datalist = root.find("PayloadDataList")
        channels = datalist.findall("Channel")

        self.payload_classes = [None] * len(channels)

        for channel in channels:
            self.parse_xml_channel(channel)

    def parse_xml_channel(self, channel_xml) -> str:
        channel_num = int(channel_xml.find("Number").text)
        field_list = []
        init_contents = []
        parse_contents = []
        fields = channel_xml.find("Fields").findall("Field")
        for field in fields:
            field_name = field.find("name").text
            field_name = field_name.lower().replace(" ", "_")
            field_type = field.find("type").text
            field_size = int(field.find("size").text)
            field_unpack = type_conv[field_type]['unpack']

            arg_str: str
            init_str: str
            parse_str: str

            if field_size == 1:
                arg_str = f"\t\t{field_name} = {type_conv[field_type]['default']},"
                init_str = f"\t\tself.{field_name} = {field_name}"
                parse_str = f"\t\tself.{field_name} = struct.unpack_from('{field_unpack}', buf, offset)[0]"
                parse_str += f"\n\t\toffset = offset + struct.calcsize('{field_unpack}')"
            else:
                arg_str = f"\t\t{field_name} = [None] * {field_size},"

                init_str = f"\t\tif len({field_name}) != {field_size}:"
                init_str += f"\n\t\t\traise ValueError('invalid {field_name} length: expected={field_size}')"
                init_str += f"\n\t\tself.{field_name} = list({field_name})"

                parse_str = f"\t\tfor i in range(0, {field_size}):"
                parse_str += f"\n\t\t\tself.{field_name}.append(struct.unpack_from('{field_unpack}', buf, offset)[0])"
                parse_str += f"\n\t\t\toffset = offset + struct.calcsize('{field_unpack}')"
            field_list.append(arg_str)
            init_contents.append(init_str)
            parse_contents.append(parse_str)

        class_name = f"CustomUserPayload{channel_num}"
        field_list_str = "\n".join(field_list)
        init_contents_str = "\n".join(init_contents)
        parse_contents_str = "\n".join(parse_contents)

        class_template = template.format(
            class_name=class_name,
            field_list=field_list_str,
            init_contents=init_contents_str,
            parse_contents=parse_contents_str)

        namespace = {}
        exec(class_template, globals(), namespace)
        self.payload_classes[channel_num] = namespace[class_name]


template = '''
from enum import Enum
import struct
import sys

class {class_name}:
	SIZE = 64

	def __init__(
        self,
{field_list}
    ):
{init_contents}

	def parse(self, buf):
		if (len(buf) != self.SIZE):
			raise BufferError('INVALID PACKET SIZE [{class_name}]: Expected=' + str(self.SIZE) + ' Received='+ str(len(buf)))

		offset = 0
{parse_contents}

	def getSize(self):
		return self.SIZE

	#def serialize(self):
        # TODO

'''
