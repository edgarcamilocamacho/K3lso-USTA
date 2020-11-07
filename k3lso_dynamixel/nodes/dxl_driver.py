from dynamixel_sdk import *
import math
import yaml

ADDR_PRESENT_POSITION = 132 
LEN_PRESENT_POSITION = 4
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4

ADDR_TORQUE_ENABLE = 64
ADDR_PWM_LIMIT = 36

PROTOCOL_VERSION = 2.0

class DxlDriver():

    def __init__(self, port, baudrate, dxl_info, initial_fake_pose, only_monitor, fake_mode):
        self.dxl_info = dxl_info
        self.initial_fake_pose = initial_fake_pose
        self.only_monitor = only_monitor
        self.fake_mode = fake_mode
        
        for motor_name in self.dxl_info:
            self.dxl_info[motor_name]['position'] = 0.0
            self.dxl_info[motor_name]['goal_position'] = 0.0

        self.torque_enable = not self.only_monitor

        if not self.fake_mode:
            self.portHandler = PortHandler(port)
            self.packetHandler = PacketHandler(PROTOCOL_VERSION)
            self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

            if not self.portHandler.openPort():
                self.error("Failed to open the port"); return
            if not self.portHandler.setBaudRate(baudrate):
                self.error("Failed to change the baudrate"); return

            for motor_name in self.dxl_info:
                m_id = self.dxl_info[motor_name]['id']
                _, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, m_id)
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    self.error('Couldn\'n connect with motor {}'.format(m_id)); return
                self.groupSyncRead.addParam(m_id)

            for motor_name in self.dxl_info:
                m_id = self.dxl_info[motor_name]['id']
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                                                    self.portHandler, m_id, ADDR_TORQUE_ENABLE, 1 if self.torque_enable else 0)
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    self.error('Couldn\'n set torquer to motor {}'.format(m_id)); return

        if self.fake_mode:
            for motor_name in self.dxl_info:
                self.dxl_info[motor_name]['position'] = self.initial_fake_pose[motor_name]

        self.first_read = False
        self.error_str = ""
        self.correct = True
    
    def error(self, msg):
        self.error_str = msg
        self.correct = False
    
    def set_goal_position(self, joint, pos):
        self.dxl_info[joint]['goal_position'] = pos

    def read(self):
        if not self.fake_mode:
            dxl_comm_result = self.groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                self.error_str = self.packetHandler.getTxRxResult(dxl_comm_result)
                return False
            for motor_name in self.dxl_info:
                motor = self.dxl_info[motor_name]
                pos = math.pi * ( ( float(self.groupSyncRead.getData(motor['id'], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) - 2048 ) / 2048 )
                if motor['invert']:
                    pos *= -1 
                pos -= motor['offset']
                pos *= motor['ratio']
                motor['position'] = pos
        if not self.first_read:
            self.first_read = True
            for motor_name in self.dxl_info:
                self.dxl_info[motor_name]['goal_position'] = self.dxl_info[motor_name]['position']
        return True

    def write(self):
        if not self.first_read:
            self.error_str = 'Before writing, it must read at least once.'
            return True
        if not self.fake_mode:
            if self.only_monitor:
                return True
            else:
                for motor_name in self.dxl_info:
                    motor = self.dxl_info[motor_name]
                    motor_pos = motor['goal_position']
                    motor_pos /= motor['ratio']
                    motor_pos += motor['offset']
                    if motor['invert']:
                        motor_pos *= -1 
                    motor_pos_int = int( ( (motor_pos / math.pi) * 2048 ) + 2048 )
                    param_goal_position = [ DXL_LOBYTE(DXL_LOWORD(motor_pos_int)), 
                                            DXL_HIBYTE(DXL_LOWORD(motor_pos_int)), 
                                            DXL_LOBYTE(DXL_HIWORD(motor_pos_int)), 
                                            DXL_HIBYTE(DXL_HIWORD(motor_pos_int))]
                    self.groupSyncWrite.addParam(motor['id'], param_goal_position)
                dxl_comm_result = self.groupSyncWrite.txPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    self.error_str = self.packetHandler.getTxRxResult(dxl_comm_result)
                    self.groupSyncWrite.clearParam()
                    return False
                self.groupSyncWrite.clearParam()
        else:
            for motor_name in self.dxl_info:
                self.dxl_info[motor_name]['position'] = self.dxl_info[motor_name]['goal_position']
        return True
    
    def get_names_positions(self):
        names = [motor_name for motor_name in self.dxl_info]
        positions = [self.dxl_info[motor_name]['position'] for motor_name in self.dxl_info]
        return names, positions
    
    def get_motor_position(self, motor_name):
        return self.dxl_info[motor_name]['position']