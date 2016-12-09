from NatNetClient import NatNetClient

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
#def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
#                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
#    print( "Received frame", frameNumber )

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    #print( "Received frame for rigid body", position)
	print(str(id) + " : ", position)
	#print(str(rotation[0] * (180/3.14)) + " " + str(rotation[1] * (180/3.14)) + " " + str(rotation[2] * (180/3.14)) + " " + str(rotation[3] * (180/3.14)))
	#print(str(id) + " : ", rotation)

# This will create a new NatNet client
streamingClient = NatNetClient()

# Configure the streaming client to call our rigid body handler on the emulator to send data out.
#streamingClient.newFrameListener = receiveNewFrame
streamingClient.newFrameListener = None
streamingClient.rigidBodyListener = receiveRigidBodyFrame

# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
streamingClient.run()
