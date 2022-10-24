#!/usr/bin/env python3

import asyncio
import sys
import csv
import random
import requests
import pymysql

import pandas

from multiprocessing import Process
from mavsdk import System
from mavsdk.mission_raw import MissionItem


async def run(drone_num, drone_type):
    drone = System(None, 50051+drone_num)
    port = 24540 + drone_num
    print(f'Connecting to drone {drone_num} by port {port}')
    await drone.connect(system_address=f"udp://:{port}")

    print(f"Waiting for drone to connect drone {drone_num}...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone {drone_num}!")
            break

    await asyncio.sleep(1)
    point = ['A', 'B', 'C', 'D', 'E']

    async for position in drone.telemetry.position():
        start_position = (float(position.latitude_deg), float(position.longitude_deg))
        break

    start_point = None
    for i in point:
        path = f'./conf/point_{i}.csv'
        f = open(path, 'r')
        rdr = csv.reader(f)
        x_pos = list()
        y_pos = list()
        for line in rdr:
            x_pos.append(float(line[0]))
            y_pos.append(float(line[1]))
        tr = (max(x_pos), max(y_pos))
        bl = (min(x_pos), min(y_pos))

        if is_inside(bl, tr, start_position):
            start_point = i
            break
        f.close()
        await asyncio.sleep(1)

    print(f'drone{drone_num} is in point {start_point}')
    mission_items = []

    dst = random.randint(0, 4)

    while point[dst] == start_point:
        dst = random.randint(0, 4)
        await asyncio.sleep(0.001)

    path = f"./conf/dst_{point[dst]}.csv"
    row = drone_num
    dataset = pandas.read_csv(path, header=None)
    lat = float(dataset.loc[row][0])
    lon = float(dataset.loc[row][1])

    await drone.mission_raw.clear_mission()

    mission_items.append(make_takeoff_mission(start_position[0], start_position[1]))
    mission_items.append(make_waypoint_mission(1, lat, lon))
    mission_items.append(make_land_mission(2, lat, lon))

    print(f"-- Uploading mission to drone {drone_num} (to point{point[dst]} line num {row} ({lat}, {lon})was at {start_point})")
    await drone.mission_raw.upload_mission(mission_items)

    async for in_air in drone.telemetry.in_air():
        await asyncio.sleep(0.01)
        if in_air:
            break

    prev_mission_group = start_point

    while True:
        prev_mission_group = await land_disarm(drone, drone_num, drone_type, prev_mission_group, point)
        async for in_air in drone.telemetry.in_air():
            await asyncio.sleep(0.01)
            if in_air:
                break
        await asyncio.sleep(1)
    return


async def land_disarm(drone, drone_num, drone_type, prev_mission_group, point):
    async for in_air in drone.telemetry.in_air():
        await asyncio.sleep(0.01)
        if not in_air:
            print(f"drone {drone_num} mission finished")
            break

    await drone.mission_raw.clear_mission()
    entries = await drone.log_files.get_entries()
    await download_log(drone, entries[-1], drone_num)
    return await give_random_mission(drone, drone_type, drone_num, prev_mission_group, point)


async def download_log(drone, entry, drone_num):
    date_without_colon = entry.date.replace(":", "-")
    filename = f"./log/A/{drone_num}-{date_without_colon}.ulog"
    print(f"drone {drone_num} Downloading: log {entry.id} from {entry.date} to {filename}")
    await drone.log_files.download_log_file(entry, filename)
    print(f"drone {drone_num} log Download complete")
    await drone.log_files.erase_all_log_files()
    files = open(filename, 'rb')
    upload = {'filearg': files}
    post_data = {"description": '', "feedback": '', "email": "", "type": "personal"}

    res = requests.post('http://localhost:5006/upload', files=upload, data=post_data)
    url = res.url
    droneid = drone_num + 1

    conn = pymysql.connect(host='125.6.40.93', user='suvlab', password='suvlab', db='suvlab', charset='utf8mb4')
    cur = conn.cursor()

    cur.execute(f"select * from log where droneid = %s", droneid)
    row_result = cur.rowcount
    if row_result == 1:
        cur.execute("UPDATE log SET url = %s WHERE droneid = %s", (url, droneid))
    else:
        cur.execute("INSERT INTO log (droneid, url) VALUES (%s, %s)", (droneid, url))
    conn.commit()
    conn.close()


async def give_random_mission(drone, drone_type, drone_num, prev_mission_group, point):
    mission_items = []

    dst = random.randint(0, 4)

    while point[dst] == prev_mission_group:
        dst = random.randint(0, 4)
        await asyncio.sleep(0.001)

    path = f"./conf/dst_{point[dst]}.csv"
    row = drone_num
    dataset = pandas.read_csv(path, header=None)
    lat = float(dataset.loc[row][0])
    lon = float(dataset.loc[row][1])
    async for position in drone.telemetry.position():
        takeoff_position = (float(position.latitude_deg), float(position.longitude_deg))
        break


    mission_items.append(make_takeoff_mission(takeoff_position[0], takeoff_position[1]))
    mission_items.append(make_waypoint_mission(1, lat, lon))
    mission_items.append(make_land_mission(2, lat, lon))

    print(f"-- Uploading mission to drone {drone_num} (to point{point[dst]}({lat}, {lon}) line num {row} was at {prev_mission_group})")
    await drone.mission_raw.upload_mission(mission_items)

    return point[dst]


def make_takeoff_mission(lat, lon, alt=360):
    return MissionItem(seq=0,
                       frame=0,
                       command=22,
                       current=1,
                       autocontinue=1,
                       param1=0.0,
                       param2=0.0,
                       param3=0.0,
                       param4=float('nan'),
                       x=int(lat * 10 ** 7),
                       y=int(lon * 10 ** 7),
                       z=alt,
                       mission_type=0)


def make_waypoint_mission(seq, lat, lon, alt=360):
    return MissionItem(seq=seq,
                       frame=0,
                       command=16,
                       current=0,
                       autocontinue=1,
                       param1=0.0,
                       param2=0.0,
                       param3=0.0,
                       param4=float('nan'),
                       x=int(lat * 10 ** 7),
                       y=int(lon * 10 ** 7),
                       z=alt,
                       mission_type=0)


def make_land_mission(seq, lat, lon, alt=360):
    return MissionItem(seq=seq,
                       frame=0,
                       command=21,
                       current=0,
                       autocontinue=1,
                       param1=0.0,
                       param2=0.0,
                       param3=0.0,
                       param4=float('nan'),
                       x=int(lat * 10 ** 7),
                       y=int(lon * 10 ** 7),
                       z=alt,
                       mission_type=0)


def is_inside(bl, tr, p):
    if bl[0] < p[0] < tr[0] and bl[1] < p[1] < tr[1]:
        return True
    else:
        return False


async def check_in_air(drone):
 async for in_air in drone.telemetry.in_air():
     return in_air


def main(drone_num, drone_type):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(drone_num, drone_type))


if __name__ == "__main__":
    drone_type = 'iris'
    drone_num = int(sys.argv[1])
    each_case = []
    for i in range(drone_num):
        p = Process(target=main, args=(i, drone_type, ))
        each_case.append(p)
    for case in each_case:
        case.start()
