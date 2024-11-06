
#region GM
from pinkylib import TCP_IP_Communication as wlcom
from pinkylib import Data_Process as dp
import time
import threading

data_from_gm = [0]
data_to_gm = [0]

def Get_Data_FROM_GM():
    global data_from_gm
    return data_from_gm

def Get_Data_TO_GM():
    global data_to_gm
    return data_to_gm

'''
어떤 데이터가 들어오든, SET은 무조건 list로 변환시킬 예정
'''
def Set_Data_FROM_GM(data):
    global data_from_gm
    if(str(type(data)) == "<class 'str'>"):
        data_from_gm = dp.Trans_Str_To_Arr(data)
    elif(str(type(data)) == "<class 'list'>"):
        data_from_gm = data
    elif(str(type(data)) == "<class 'int'>"):
        data_from_gm = [data]

def Set_Data_TO_GM(data):
    global data_to_gm
    if(str(type(data)) == "<class 'str'>"):
        data_to_gm = dp.Trans_Str_To_Arr(data)
    elif(str(type(data)) == "<class 'list'>"):
        data_to_gm = data
    elif(str(type(data)) == "<class 'int'>"):
        data_to_gm = [data]

'''
규민-서버
유원-클라이언트
'''
def Threading_Communication_Server():
    conn = wlcom.Init_Server_Socket()
    data = '0'
    while True:
        Set_Data_TO_YW(data)
        wlcom.Send_Socket(conn,data)
        time.sleep(0.05)
        Set_Data_FROM_YW(wlcom.Receive_Socket(conn))
        time.sleep(0.05)

server_thread = threading.Thread(target=Threading_Communication_Server, args=())
server_thread.start()

#endregion GM

#region YW
from pinkylib import TCP_IP_Communication as wlcom
from pinkylib import Data_Process as dp
import time
import threading

data_from_yw = [0]
data_to_yw = [0]

def Get_Data_From_YW():
    global data_from_yw
    return data_from_yw

def Get_Data_TO_YW():
    global data_to_yw
    return data_to_yw

'''
어떤 데이터가 들어오든, SET은 무조건 list로 변환시킬 예정
'''
def Set_Data_FROM_YW(data):
    global data_from_yw
    if(str(type(data)) == "<class 'str'>"):
        data_from_yw = dp.Trans_Str_To_Arr(data)
    elif(str(type(data)) == "<class 'list'>"):
        data_from_yw = data
    elif(str(type(data)) == "<class 'int'>"):
        data_from_yw = [data]

def Set_Data_TO_YW(data):
    global data_to_yw
    if(str(type(data)) == "<class 'str'>"):
        data_to_yw = dp.Trans_Str_To_Arr(data)
    elif(str(type(data)) == "<class 'list'>"):
        data_to_yw = data
    elif(str(type(data)) == "<class 'int'>"):
        data_to_yw = [data]
        
'''
규민-서버
유원-클라이언트
'''
def Threading_Communication_Client():
    s = wlcom.Init_Client_Socket('10.211.173.2')
    data = '0'
    while True:
        Set_Data_FROM_GM(wlcom.Receive_Socket(s))
        time.sleep(0.05)
        Set_Data_TO_GM(data)
        wlcom.Send_Socket(s, data)
        time.sleep(0.05)

client_thread = threading.Thread(target=Threading_Communication_Client, args=())
client_thread.start()

#endregion YW
