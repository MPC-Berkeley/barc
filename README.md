<h1>Berkeley Autonomous Race Car (BARC) Repo</h1>

<p><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">The Berkeley Autonomous Race Car is a development platform for autonomous driving to achieve complex maneuvers such as drifting, lane changes, and obstacle avoidance. A 1/10 scale RC car and an embedded Linux computer make up the hardware platform of the project. This project aims to be fully open-source. The data collection process is cloud-based and brings new dimensions to the Vehicle Dynamics and Control Theory teaching and research world.</span></p>

<p><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">This site is home to the repository. The main folders in this repo include</span></p>

<ul>
	<li><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">CAD</span>
	<ul>
		<li><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">​STL and DWX files for fabricating the aluminum deck and side brackets from for the vehicle, and the ABS cover for the odroid </span></li>
	</ul>
	</li>
	<li><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">Dator</span>
	<ul>
		<li><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">​Web server for cloud robotics. Provides a standard way to record data and events from one or more local computers for later analysis. Based <a href="https://github.com/bwootton/Dator">this repo</a> from Bruce Wooton</span></li>
	</ul>
	</li>
	<li><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">Arduino</span>
	<ul>
		<li><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">​Files to program the arduino to (1) send commands to the electronic speed control (ESC) unit and the servo, and to (2) acquire measurements from the encoders and ultrasound sensors</span></li>
	</ul>
	</li>
	<li><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">Scripts</span>
	<ul>
		<li><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">​​​​Bash programs that set up environment variables and launch the local server upon boot</span></li>
	</ul>
	</li>
	<li><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">Utility</span>
	<ul>
		<li><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">​Miscellaneous useful scripts</span></li>
	</ul>
	</li>
	<li><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">Workspace</span>
	<ul>
		<li><span style="color: rgba(26, 26, 26, 0.701961); font-family: proxima-nova; font-size: 18px; line-height: 28.8px; text-align: center;">​Robotic Operating System (ROS) workspace that contains the barc package. This package holds the source code to control the vehicle using the ROS framework</span></li>
	</ul>
	</li>
</ul>
