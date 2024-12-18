# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: tracking_data.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='tracking_data.proto',
  package='simple_ackermann_proto',
  syntax='proto3',
  serialized_pb=_b('\n\x13tracking_data.proto\x12\x16simple_ackermann_proto\"U\n\x05State\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\x12\r\n\x05theta\x18\x03 \x01(\x01\x12\t\n\x01v\x18\x04 \x01(\x01\x12\r\n\x05omega\x18\x05 \x01(\x01\x12\r\n\x05kappa\x18\x06 \x01(\x01\"8\n\rControlSignal\x12\t\n\x01v\x18\x01 \x01(\x01\x12\r\n\x05omega\x18\x02 \x01(\x01\x12\r\n\x05kappa\x18\x03 \x01(\x01\"\xf9\x01\n\x08ParamMPC\x12\x0f\n\x07horizon\x18\x01 \x01(\r\x12\x10\n\x08interval\x18\x02 \x01(\x01\x12\x11\n\tstate_dim\x18\x03 \x01(\r\x12\x11\n\tinput_dim\x18\x04 \x01(\r\x12\x0f\n\x07max_vel\x18\x05 \x01(\x01\x12\x0f\n\x07min_vel\x18\x06 \x01(\x01\x12\x0f\n\x07max_acc\x18\x07 \x01(\x01\x12\x0f\n\x07min_acc\x18\x08 \x01(\x01\x12\x1e\n\x16steer_angle_rate_limit\x18\t \x01(\x01\x12\x17\n\x0fmin_turn_radius\x18\n \x01(\x01\x12\x13\n\x0btrack_width\x18\x0b \x01(\x01\x12\x12\n\nwheel_base\x18\x0c \x01(\x01\"\x90\x02\n\x0cTrackingData\x12\x0e\n\x06length\x18\x01 \x01(\r\x12\x11\n\ttimestamp\x18\x02 \x03(\t\x12\x35\n\x0ereference_data\x18\x03 \x03(\x0b\x32\x1d.simple_ackermann_proto.State\x12\x32\n\x0b\x61\x63tual_data\x18\x04 \x03(\x0b\x32\x1d.simple_ackermann_proto.State\x12=\n\x0e\x63ontrol_signal\x18\x05 \x03(\x0b\x32%.simple_ackermann_proto.ControlSignal\x12\x33\n\tmpc_param\x18\x06 \x01(\x0b\x32 .simple_ackermann_proto.ParamMPCb\x06proto3')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_STATE = _descriptor.Descriptor(
  name='State',
  full_name='simple_ackermann_proto.State',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='simple_ackermann_proto.State.x', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='simple_ackermann_proto.State.y', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='theta', full_name='simple_ackermann_proto.State.theta', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='v', full_name='simple_ackermann_proto.State.v', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='omega', full_name='simple_ackermann_proto.State.omega', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='kappa', full_name='simple_ackermann_proto.State.kappa', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=47,
  serialized_end=132,
)


_CONTROLSIGNAL = _descriptor.Descriptor(
  name='ControlSignal',
  full_name='simple_ackermann_proto.ControlSignal',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='v', full_name='simple_ackermann_proto.ControlSignal.v', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='omega', full_name='simple_ackermann_proto.ControlSignal.omega', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='kappa', full_name='simple_ackermann_proto.ControlSignal.kappa', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=134,
  serialized_end=190,
)


_PARAMMPC = _descriptor.Descriptor(
  name='ParamMPC',
  full_name='simple_ackermann_proto.ParamMPC',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='horizon', full_name='simple_ackermann_proto.ParamMPC.horizon', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='interval', full_name='simple_ackermann_proto.ParamMPC.interval', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='state_dim', full_name='simple_ackermann_proto.ParamMPC.state_dim', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='input_dim', full_name='simple_ackermann_proto.ParamMPC.input_dim', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_vel', full_name='simple_ackermann_proto.ParamMPC.max_vel', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='min_vel', full_name='simple_ackermann_proto.ParamMPC.min_vel', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_acc', full_name='simple_ackermann_proto.ParamMPC.max_acc', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='min_acc', full_name='simple_ackermann_proto.ParamMPC.min_acc', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='steer_angle_rate_limit', full_name='simple_ackermann_proto.ParamMPC.steer_angle_rate_limit', index=8,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='min_turn_radius', full_name='simple_ackermann_proto.ParamMPC.min_turn_radius', index=9,
      number=10, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='track_width', full_name='simple_ackermann_proto.ParamMPC.track_width', index=10,
      number=11, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='wheel_base', full_name='simple_ackermann_proto.ParamMPC.wheel_base', index=11,
      number=12, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=193,
  serialized_end=442,
)


_TRACKINGDATA = _descriptor.Descriptor(
  name='TrackingData',
  full_name='simple_ackermann_proto.TrackingData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='length', full_name='simple_ackermann_proto.TrackingData.length', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='simple_ackermann_proto.TrackingData.timestamp', index=1,
      number=2, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='reference_data', full_name='simple_ackermann_proto.TrackingData.reference_data', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='actual_data', full_name='simple_ackermann_proto.TrackingData.actual_data', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='control_signal', full_name='simple_ackermann_proto.TrackingData.control_signal', index=4,
      number=5, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='mpc_param', full_name='simple_ackermann_proto.TrackingData.mpc_param', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=445,
  serialized_end=717,
)

_TRACKINGDATA.fields_by_name['reference_data'].message_type = _STATE
_TRACKINGDATA.fields_by_name['actual_data'].message_type = _STATE
_TRACKINGDATA.fields_by_name['control_signal'].message_type = _CONTROLSIGNAL
_TRACKINGDATA.fields_by_name['mpc_param'].message_type = _PARAMMPC
DESCRIPTOR.message_types_by_name['State'] = _STATE
DESCRIPTOR.message_types_by_name['ControlSignal'] = _CONTROLSIGNAL
DESCRIPTOR.message_types_by_name['ParamMPC'] = _PARAMMPC
DESCRIPTOR.message_types_by_name['TrackingData'] = _TRACKINGDATA

State = _reflection.GeneratedProtocolMessageType('State', (_message.Message,), dict(
  DESCRIPTOR = _STATE,
  __module__ = 'tracking_data_pb2'
  # @@protoc_insertion_point(class_scope:simple_ackermann_proto.State)
  ))
_sym_db.RegisterMessage(State)

ControlSignal = _reflection.GeneratedProtocolMessageType('ControlSignal', (_message.Message,), dict(
  DESCRIPTOR = _CONTROLSIGNAL,
  __module__ = 'tracking_data_pb2'
  # @@protoc_insertion_point(class_scope:simple_ackermann_proto.ControlSignal)
  ))
_sym_db.RegisterMessage(ControlSignal)

ParamMPC = _reflection.GeneratedProtocolMessageType('ParamMPC', (_message.Message,), dict(
  DESCRIPTOR = _PARAMMPC,
  __module__ = 'tracking_data_pb2'
  # @@protoc_insertion_point(class_scope:simple_ackermann_proto.ParamMPC)
  ))
_sym_db.RegisterMessage(ParamMPC)

TrackingData = _reflection.GeneratedProtocolMessageType('TrackingData', (_message.Message,), dict(
  DESCRIPTOR = _TRACKINGDATA,
  __module__ = 'tracking_data_pb2'
  # @@protoc_insertion_point(class_scope:simple_ackermann_proto.TrackingData)
  ))
_sym_db.RegisterMessage(TrackingData)


# @@protoc_insertion_point(module_scope)
