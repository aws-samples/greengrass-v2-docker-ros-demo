# ROS2 Docker Sample Application with AWS IoT Greengrass 2.0

This sample application demonstrates how to deploy and run ROS applications with IoT Greengrass 2.0 and Docker. The application will run three containers using a docker compose file. Two of the containers, a talker and a listener, will use local ROS messaging to send and receive a “Hello World” message over the ROS topic /chatter. A third container will use the Greengrass SDK to bridge messages published over the ROS topic /chatter with a local socket used by Greengrass for inter-process communication between components. Greengrass will then relay the message over an MQTT topic named chatter in the cloud. Here is an architecture diagram that shows what we will build:

![Blog Architecture](/images/blog-architecture.png)

To get started, clone this repository.

```
git clone https://github.com/aws-samples/greengrass-v2-docker-ros-demo.git ~/greengrass-v2-docker-ros-demo
```

## Prerequsites

Before continuing, make sure your development environment and deployment target (your robot or any Linux-based machine) has the following dependencies

### Software and Tools Dependencies
On the *development machine* (Your laptop or IDE, ex: AWS RoboMaker IDE):

- **AWS CLI** configured with elevated permissions:
  ```bash
  curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
  unzip awscliv2.zip
  sudo ./aws/install
  ```
  [Click here for full instructions](https://docs.aws.amazon.com/cli/latest/userguide/install-cliv2.html).

- **Docker** and **Docker Compose**:
  - Follow the docker installation steps per [these instructions](https://docs.docker.com/engine/install/ubuntu/)
  - Run these commands to install Docker Compose. For full instructions, [click here](https://docs.docker.com/compose/install/).
   ```bash
      sudo curl -L "https://github.com/docker/compose/releases/download/1.29.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
      sudo chmod +x /usr/local/bin/docker-compose
   ```

On the *deployment target* (the robot):
- **Java Runtime Environment**:
 ```
 apt-get update
 apt-get install default-jre
 ```
- **Docker** and **Docker Compose** (same as above).

### AWS Resource Dependencies

You will also need the following AWS resources:
-	An **S3 bucket** to stage deployment artifacts .
-	A **IAM Greengrass Provisioning User** with [minimal IAM](https://docs.aws.amazon.com/greengrass/v2/developerguide/provision-minimal-iam-policy.html) access to provision new robots.
-	A base **IAM Role** to provide robots access to specific AWS resources.

There is a CloudFormation template that will create the above AWS resources [here](greengrass/greengrass_bootstrap.template.yaml). To launch the stack, run the following command after cloning this respository.

```
cd ~/greengrass-v2-docker-ros-demo
aws cloudformation create-stack --stack-name GG-Provisioning --template-body file://greengrass/greengrass_bootstrap.template.yaml --capabilities CAPABILITY_NAMED_IAM
```

***Important**: If you decide to use the *Greengrass Provisioning User* created by the cloudformation template above, you will also need to create a set of access credentials (AWS ACCESS KEY ID, and AWS SECRET ACCESS KEY) to use in the provisioning step, per [these instructions.](https://docs.aws.amazon.com/IAM/latest/UserGuide/id_credentials_access-keys.html)*

## Build Docker Image and Upload to Amazon ECR

Clone this repository and run the build command. It will use the Dockerfile and docker compose YAML file to build the container image.

1. On the *development machine*:

  ```
  cd ~/greengrass-v2-docker-ros-demo
  DOCKER_BUILDKIT=1 docker-compose build
  ```

2. Create a new Amazon ECR Repository:

  ```
  aws ecr create-repository --repository-name ros-foxy-greengrass-demo | grep repositoryUri
  ```

3. The response will look something like this. Copy the URI. 

  > "repositoryUri": "1234567819.dkr.ecr.us-east-1.amazonaws.com/ros-foxy-greengrass-demo",

4. Sign into ECR and upload the new image (replace the ACCOUT_ID and REGION placeholders with your values from the URI above):

  ```
  aws ecr get-login-password --region <REGION> | docker login --username AWS --password-stdin <ACCOUNT_ID>.dkr.ecr.us-east-1.amazonaws.com

  docker tag ros-foxy-greengrass-demo:latest <ACCOUNT_ID>.dkr.ecr.<REGION>.amazonaws.com/ ros-foxy-greengrass-demo:latest
  docker push <ACCOUNT_ID>.dkr.ecr.<REGION>.amazonaws.com/ros-foxy-greengrass-demo:latest
  ```

5. Copy/paste the created docker image in ECR (*<ACCOUNT_ID>.dkr.ecr.<REGION>.amazonaws.com/ros-foxy-greengrass-demo:latest*) into the following files, replacing the placeholder `<YOUR_PRIVATE_ECR_IMAGE_ID_ROS_GREENGRASS_DEMO>`:
  - docker-compose.yaml
  - greengrass/com.example.ros2.demo/1.0.0/recipes 

## Provision Greengrass 2.0 on the Robot

On the **development** machine, connect to the robot over SSH or open a shell terminal on the *deployment target* linux-based device. Run the following commands in the SSH shell (*replace the placeholders with your AWS Access Key ID and Secret Key*):

***Important:** First ensure that the robot has **Java Runtime Environment (JRE)**, **docker** and **docker compose** installed.*

Run the following install and provisioning commands:

```
export AWS_ACCESS_KEY_ID=<INSERT_YOUR_AWS_ACCESS_KEY_ID_HERE>
export AWS_SECRET_ACCESS_KEY=<INSERT_YOUR_AWS_SECRET_KEY>

curl -s https://d2s8p88vqu9w66.cloudfront.net/releases/greengrass-nucleus-latest.zip > greengrass-nucleus-latest.zip && unzip greengrass-nucleus-latest.zip -d GreengrassCore

sudo -E java -Droot="/greengrass/v2" -Dlog.store=FILE -jar ./GreengrassCore/lib/Greengrass.jar \
           --thing-name ROS2_Sample_Robot \
           --thing-group-name ROS2_Sample_Robots \
           --component-default-user ggc_user:ggc_group \
           --provision true \
           --setup-system-service true \
           --deploy-dev-tools true
           
 sudo usermod -aG docker ggc_user
```

Keep this shell open for later. For more details on AWS IoT Greengrass service invocation, click here. 

## Create and Deploy the ROS2 Docker Component

Return to the *development machine* shell. 

1. Open the [Greengrass recipe file](greengrass/com.example.ros2.demo/1.0.0/recipes/recipe.yaml) and modify it. Change the placeholders <YOUR_BUCKET_NAME> and <YOUR_PRIVATE_ECR_IMAGE_ID_ROS_GREENGRASS_DEMO> to use an S3 bucket in your account and the ECR URI to the image you created above.
	
2. Upload the docker compose file to the same Amazon S3 bucket using the object key defined in the recipe:

  ```
  cd ~/greengrass-v2-docker-ros-demo
  aws s3 cp ./docker-compose.yaml s3://<MY_BUCKET_NAME>/com.example.ros2.demo/1.0.0/artifacts/docker-compose.yaml
  ```

3. Create the new component with the modified recipe file:

  ```
  aws greengrassv2 create-component-version \
    --inline-recipe fileb://greengrass/com.example.ros2.demo/1.0.0/recipes/recipe.yaml
  ```

4. Open the AWS IoT Greengrass console. Click on **Greengrass** > **Deployments**. Click the checkbox beside *Deployment for ROS2_Sample_Robots*, then press **Revise Deployment** in the top right of the section.
5. Follow through the deployment wizard. In **Step 2**, click the checkbox beside **com.example.ros2.demo**. Click through to the end of the wizard and press **Deploy**.
6. After a minute or two, run the following commands in the SSH shell on the deployment target (the robot) to see if the ROS containers are running.

  ```
  cd /greengrass/v2/bin/
  sudo ./greengrass-cli component list
  ```

## Monitor and interact with Greengrass Deployment

1. Use the docker compose file that was deployed to tail the logs and see the pub/sub communication between nodes. 

  ```
  export ARTIFACT_DIR=/greengrass/v2/packages/artifacts/com.example.ros2.demo/1.0.0/
  sudo docker-compose -f $ARTIFACT_DIR/docker-compose.yaml logs --follow
  ```

2. You should see your application running Hello World messages! 

    *Optional*: To stop and/or restart the ROS containers, run the below commands. To learn more about what you can do with the Greengrass CLI, [click here](https://docs.aws.amazon.com/greengrass/v2/developerguide/gg-cli-reference.html).

    **Stop a component:** 
    ```
    sudo /greengrass/v2/bin/greengrass-cli component stop -n com.example.ros2.demo
    ```

    **Check the state (it should be marked as "FINISHED" after a stop operation):**
    ```
    sudo /greengrass/v2/bin/greengrass-cli component list
    ```

    **Restart a component:**
    ```
    sudo /greengrass/v2/bin/greengrass-cli component restart -n com.example.ros2.demo
    ```

3. Open the AWS IoT console and subscribe to the MQTT topic `chatter`. “*Hello World*” messages will start to appear in AWS IoT.

## Clean-up and Uninstall


1. Follow [these instructions](https://docs.aws.amazon.com/greengrass/v2/developerguide/uninstall-greengrass-core-v2.html) to remove Greengrass v2 on the deployment target.
2. Delete the cloudformation stack you created above by following [these instructions](https://docs.aws.amazon.com/AWSCloudFormation/latest/UserGuide/cfn-console-delete-stack.html): 
3. Delete the IoT Thing and Thing Group provisioned by Greengrass in the AWS IoT console.
