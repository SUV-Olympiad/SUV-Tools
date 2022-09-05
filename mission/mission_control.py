#!/usr/bin/env python3
#have to contaion mission coordinate at ./missions

import asyncio
import csv
import random
import aiofiles
from multiprocessing import Process

from aiocsv import AsyncReader
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

async def run(drone_num, drone_type):
    drone = System(None, 50051+drone_num)
    port = 14540 + drone_num
    print(f'Connecting to drone {drone_num} by port {port}')
    await drone.connect(system_address=f"udp://:{port}")

    print(f"Waiting for drone to connect drone {drone_num}...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone {drone_num}!")
            break

    path = f"./missions/{drone_type}_{drone_num}.csv"
    print(f'reading {path}...')
    mission_items = []
    async with aiofiles.open(path, mode="r", encoding="utf-8", newline="") as afp:
        async for row in AsyncReader(afp, delimiter=","):
            if row[0].startswith('#'):
                continue
            lat = float(row[0])
            lon = float(row[1])
            height = float(row[2])
            mission_items.append(make_mission_items(lat, lon, height))
    mission_plan = MissionPlan(mission_items)


    print(f"-- Uploading mission to drone {drone_num}")
    await drone.mission.upload_mission(mission_plan)

    async for in_air in drone.telemetry.in_air():
        if in_air:
            break
        #print("waiting for drone flying")

    while True:
        await land_disarm(drone, drone_num, drone_type)

    return


async def land_disarm(drone, drone_num, drone_type):
    if await drone.mission.is_mission_finished():
        print(f"drone {drone_num} mission finished. send landing command")
        await drone.action.land()

        async for in_air in drone.telemetry.in_air():
            if not in_air:
                break

        await drone.action.disarm()
        await give_random_mission(drone, drone_type, drone_num)


async def give_random_mission(drone, drone_type, drone_num):
    path = f"./missions/{drone_type}_{random.randint(0,3)}.csv"
    print(f'reading {path}... for drone {drone_num}')
    f = open(path, 'r')
    rdr = csv.reader(f)
    mission_items = []
    for line in rdr:
        if line[0].startswith('#'):
            continue
        lat = float(line[0])
        lon = float(line[1])
        height = float(line[2])
        mission_items.append(make_mission_items(lat, lon, height))
    mission_plan = MissionPlan(mission_items)
    print(f"-- Uploading mission to drone {drone_num}")
    await drone.mission.upload_mission(mission_plan)

    await drone.action.arm()
    await drone.mission.start_mission()


def make_mission_items(lat, lon, height):
    return MissionItem(lat,
                       lon,
                       height,
                       10,
                       True,
                       float('nan'),
                       float('nan'),
                       MissionItem.CameraAction.NONE,
                       float('nan'),
                       float('nan'),
                       float('nan'),
                       float('nan'),
                       float('nan'))

async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return

def main(drone_num, drone_type):
    #loop = asyncio.get_event_loop()
    # loop.run_until_complete(run(drone_num, drone_type))
    asyncio.run(run(drone_num, drone_type))

if __name__ == "__main__":
    drone_type = 'iris'
    drone_num = 4
    each_case = []
    for i in range(drone_num):
        p = Process(target=main, args=(i, drone_type, ))
        each_case.append(p)
    for case in each_case:
        case.start()
