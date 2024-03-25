from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class GeneralStatus(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    STATUS_OK: _ClassVar[GeneralStatus]
    STATUS_FAULT: _ClassVar[GeneralStatus]

class DroneStatus(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    DRONE_STATUS_GROUNDED: _ClassVar[DroneStatus]
    DRONE_STATUS_HOVERING: _ClassVar[DroneStatus]
    DRONE_STATUS_MOVING_TO_WAYPOINT: _ClassVar[DroneStatus]
STATUS_OK: GeneralStatus
STATUS_FAULT: GeneralStatus
DRONE_STATUS_GROUNDED: DroneStatus
DRONE_STATUS_HOVERING: DroneStatus
DRONE_STATUS_MOVING_TO_WAYPOINT: DroneStatus

class Waypoint(_message.Message):
    __slots__ = ("position_long", "position_lat")
    POSITION_LONG_FIELD_NUMBER: _ClassVar[int]
    POSITION_LAT_FIELD_NUMBER: _ClassVar[int]
    position_long: float
    position_lat: float
    def __init__(self, position_long: _Optional[float] = ..., position_lat: _Optional[float] = ...) -> None: ...

class Obstacle(_message.Message):
    __slots__ = ("location", "radius_m")
    LOCATION_FIELD_NUMBER: _ClassVar[int]
    RADIUS_M_FIELD_NUMBER: _ClassVar[int]
    location: Waypoint
    radius_m: float
    def __init__(self, location: _Optional[_Union[Waypoint, _Mapping]] = ..., radius_m: _Optional[float] = ...) -> None: ...

class ToDrone(_message.Message):
    __slots__ = ("gen_status", "new_waypoint", "tare_position")
    GEN_STATUS_FIELD_NUMBER: _ClassVar[int]
    NEW_WAYPOINT_FIELD_NUMBER: _ClassVar[int]
    TARE_POSITION_FIELD_NUMBER: _ClassVar[int]
    gen_status: GeneralStatus
    new_waypoint: Waypoint
    tare_position: Waypoint
    def __init__(self, gen_status: _Optional[_Union[GeneralStatus, str]] = ..., new_waypoint: _Optional[_Union[Waypoint, _Mapping]] = ..., tare_position: _Optional[_Union[Waypoint, _Mapping]] = ...) -> None: ...

class ToSensor(_message.Message):
    __slots__ = ("gen_status", "drone_status", "current_position", "next_position", "obstacle")
    GEN_STATUS_FIELD_NUMBER: _ClassVar[int]
    DRONE_STATUS_FIELD_NUMBER: _ClassVar[int]
    CURRENT_POSITION_FIELD_NUMBER: _ClassVar[int]
    NEXT_POSITION_FIELD_NUMBER: _ClassVar[int]
    OBSTACLE_FIELD_NUMBER: _ClassVar[int]
    gen_status: GeneralStatus
    drone_status: DroneStatus
    current_position: Waypoint
    next_position: Waypoint
    obstacle: Obstacle
    def __init__(self, gen_status: _Optional[_Union[GeneralStatus, str]] = ..., drone_status: _Optional[_Union[DroneStatus, str]] = ..., current_position: _Optional[_Union[Waypoint, _Mapping]] = ..., next_position: _Optional[_Union[Waypoint, _Mapping]] = ..., obstacle: _Optional[_Union[Obstacle, _Mapping]] = ...) -> None: ...
