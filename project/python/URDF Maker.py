names = ["WRook1", "WKnight1", "WBishop1", "WQueen", "WKing", "WBishop2", "WKnight2", "WRook2",
         "WPawn1", "WPawn2", "WPawn3", "WPawn4", "WPawn5", "WPawn6", "WPawn7", "WPawn8",
         "BPawn1", "BPawn2", "BPawn3", "BPawn4", "BPawn5", "BPawn6", "BPawn7", "BPawn8",
         "BRook1", "BKnight1", "BBishop1", "BQueen", "BKing", "BBishop2", "BKnight2", "BRook2"]

files = ["WRook", "WKnight", "WBishop", "WQueen", "WKing", "WBishop", "WKnight", "WRook",
         "WPawn", "WPawn", "WPawn", "WPawn", "WPawn", "WPawn", "WPawn", "WPawn",
         "BPawn", "BPawn", "BPawn", "BPawn", "BPawn", "BPawn", "BPawn", "BPawn",
         "BRook", "BKnight", "BBishop", "BQueen", "BKing", "BBishop", "BKnight", "BRook"]


heights = [0.075, 0.09, 0.1, 0.12, 0.13, 0.1, 0.09, 0.075,
           0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06,
           0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06,
           0.075, 0.09, 0.1, 0.12, 0.13, 0.1, 0.09, 0.075]

offsets = [0.00675, 0.00675, 0.00675, 0.00675, 0.00675, 0.00675, 0.00675, 0.00675,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0.00675, 0.00675, 0.00675, 0.00675, 0.00675, 0.00675, 0.00675, 0.00675]

cx = 0.4 + 0.48*3.5/8
cy = - 0.48*3.5/8
k = 0.48/8

x = 0

for i in range(0, 32):
    y = i%8
    print("<dynamic_object name=\"",names[i],"\">", sep="")
    print("		<origin xyz=\"",cx - x * k," ",cy + y * k," 0\" rpy=\"0 0 0\" />", sep="")
    print("		<inertial>")
    print("        		<origin xyz=\"0 0 ",heights[i]/2,"\" rpy=\"0 0 0\" />")
    print("        		<mass value=\"0.050\" />")
    print("        		<inertia ixx=\"0.0000379\" iyy=\"0.0000379\" izz=\"0.0000225\" ixy=\"0\" ixz=\"0\" iyz=\"0\" />")
    print("      	</inertial>")
    print("")
    print("	    <visual>")
    print("	        <origin xyz=\"0 0 ",offsets[i],"\" rpy=\"1.5708 0 -1.5708\" />", sep="")
    print("		<geometry>")
    print("	            <mesh filename=\"../../model/chess/meshes/visual/",files[i],".obj\" scale=\"0.04 0.04 0.04\"/>", sep="")
    print("	        </geometry>")
    print("	        <material name=\"material\">")
    print("		        <color rgba=\"0 0 0 1.0\" />")
    print("	        </material>")
    print("	    </visual>")
    print("")
    #print("	    <visual>")
    #print("	        <origin xyz=\"0 0 0.03\" rpy=\"0 0 0\" />")
    #print("	        <geometry>")
    #print("	            <box size=\"0.03 0.03 0.06\" />")
    #print("	        </geometry>")
    #print("	        <material name=\"material\">")
    #print("		        <color rgba=\"1 1 1 1.0\" />")
    #print("	        </material>")
    #print("	    </visual>")
    print("")
    print("	    <collision>")
    print("	        <origin xyz=\"0 0 ",heights[i]/2,"\" rpy=\"0 0 0\" />", sep="")
    print("	        <geometry>")
    print("	            <box size=\"0.03 0.03 ",heights[i],"\" />", sep="")
    print("	        </geometry>")
    #print("	        <material name=\"material\">")
    #print("		    <color rgba=\"1 1 1 1.0\" />")
    #print("	        </material>")
    print("	    </collision>")
    print("	</dynamic_object>")
    print("")
    if (y ==7):
        if (x == 1):
            x = 6
        else:
            x = x+1
