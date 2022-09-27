#!/usr/bin/env python3

import asyncio
import sys
import csv
import random

import pandas

from multiprocessing import Process
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

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
    row = random.randint(0, 3)
    # print(f'reading {path}... {row}')
    dataset = pandas.read_csv(path, header=None)
    lat = float(dataset.loc[row][0])
    lon = float(dataset.loc[row][1])
    mission_items.append(make_mission_items(start_position[0], start_position[1]))
    mission_items.append(make_mission_items(lat, lon))
    mission_plan = MissionPlan(mission_items)

    print(f"-- Uploading mission to drone {drone_num} (to point{point[dst]} line num {row} ({lat}, {lon})was at {start_point})")
    await drone.mission.upload_mission(mission_plan)

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
    while True:
        if await drone.mission.is_mission_finished():
            print(f"drone {drone_num} mission finished. send landing command")
            await drone.action.land()

            async for in_air in drone.telemetry.in_air():
                await asyncio.sleep(0.01)
                if not in_air:
                    break

            await drone.action.disarm()
            entries = await drone.log_files.get_entries()
            await download_log(drone, entries[-1], drone_num)
            return await give_random_mission(drone, drone_type, drone_num, prev_mission_group, point)
        await asyncio.sleep(0.01)


async def download_log(drone, entry, drone_num):
    date_without_colon = entry.date.replace(":", "-")
    filename = f"./log/A/{drone_num}-{date_without_colon}.ulog"
    print(f"Downloading: log {entry.id} from {entry.date} to {filename}")
    await drone.log_files.download_log_file(entry, filename)
    print(f"drone {drone_num} log Download complete")
    await drone.log_files.erase_all_log_files()


async def give_random_mission(drone, drone_type, drone_num, prev_mission_group, point):
    mission_items = []

    dst = random.randint(0, 4)

    while point[dst] == prev_mission_group:
        dst = random.randint(0, 4)
        await asyncio.sleep(0.001)

    path = f"./conf/dst_{point[dst]}.csv"
    row = random.randint(0, 3)
    dataset = pandas.read_csv(path, header=None)
    lat = float(dataset.loc[row][0])
    lon = float(dataset.loc[row][1])
    async for position in drone.telemetry.position():
        takeoff_position = (float(position.latitude_deg), float(position.longitude_deg))
        break
    mission_items.append(make_mission_items(takeoff_position[0], takeoff_position[1]))
    mission_items.append(make_mission_items(lat, lon))
    mission_plan = MissionPlan(mission_items)
    print(f"-- Uploading mission to drone {drone_num} (to point{point[dst]}({lat}, {lon}) line num {row} was at {prev_mission_group})")
    await drone.mission.upload_mission(mission_plan)

    await drone.action.arm()
    await drone.mission.start_mission()

    return point[dst]


def make_mission_items(lat, lon):
    return MissionItem(lat,
                       lon,
                       305,
                       100,
                       True,
                       float('nan'),
                       float('nan'),
                       MissionItem.CameraAction.NONE,
                       float('nan'),
                       float('nan'),
                       float('nan'),
                       float('nan'),
                       float('nan'))


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