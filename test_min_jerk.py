import wpoint_nav
import time

if __name__ =='__main__':

    drone1 = wpoint_nav.DJI_min_jerk_nav()

    drone1.set_local_ref_service()

    sdkt = drone1.sdkActivation(True)
    print("sdk",sdkt)

    takeof = drone1.takeoff()
    print("takeoff",takeof)
    time.sleep(1)

    waypoint = drone1.move2wpoint([[5.618631475728761, 3.0709901237141803, 15], [9.7861877397350892, 26.706376196007174, 15], [19.63426526985717, 24.969894419337869, 15], [15.466709005850843, 1.3345083470448773, 15], [25.314786535972921, -0.40197342962442617, 15], [29.48234279997925, 23.233412642668565, 15], [39.330420330101333, 21.496930865999261, 15], [35.162864066095004, -2.1384552062937296, 15]])
    print(waypoint)
	
    landing = drone1.land()
    print('landing',landing)

    sdkf = drone1.sdkActivation(False)
    print('sdkF',sdkf)
    
    print("Completed Sir")