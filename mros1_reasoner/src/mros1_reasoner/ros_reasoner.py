import rospy

import actionlib
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from metacontrol_msgs.msg import MvpReconfigurationAction, MvpReconfigurationGoal, \
                                GraphManipulationActionAction,  GraphManipulationActionGoal, \
                                GraphManipulationMessage, SystemState
from metacontrol_msgs.srv import QAPredictions

from mros1_reasoner.reasoner import Reasoner
from mros1_reasoner.tomasys import obtainBestFunctionDesign, print_ontology_status, evaluateObjectives, resetKBstatuses, test_desired_config
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import String

class RosReasoner(Reasoner):
    """docstring for RosComponents."""

    def __init__(self):
        super(RosReasoner, self).__init__()
        self.initialized = False
        # Start rosnode
        rospy.init_node('mros1_reasoner_node', anonymous=True)

        #### Read ROS parameters
        # Get ontology and tomasys file paths from parameters
        model_file = self.check_and_read_parameter('~model_file')
        tomasys_file =  self.check_and_read_parameter('~tomasys_file')
        # Get desired_configuration_name from parameters
        self.desired_configuration = self.check_and_read_parameter('/desired_configuration')
        self.grounded_configuration = self.check_and_read_parameter('/desired_configuration')

        # Get modification number
        self.modification = self.check_and_read_parameter('/modification',1)

        #Start interfaces
        sub_diagnostics = rospy.Subscriber('/diagnostics', DiagnosticArray, self.callbackDiagnostics)

        self.rosgraph_manipulator_client = actionlib.SimpleActionClient(
                'rosgraph_manipulator_action_server',
                MvpReconfigurationAction)
        # rosgraph_manipulator_client.wait_for_server()

        # load tomasys
        if tomasys_file is not None:
            self.load_tomasys_from_file(tomasys_file)
            if self.tomasys is not None:
                rospy.loginfo("Loaded tomasys: %s", str(tomasys_file))
            else:
                rospy.logerr("Failed to load tomasys from: %s", str(tomasys_file))
                return
        else:
                return

        rospy.sleep(0.5) # Wait for subscribers (only for the test_1_level_functional_architecture)

        # load ontology
        if model_file is not None:
            self.load_onto_from_file(model_file)
            if self.onto is not None:
                rospy.loginfo("Loaded ontology: %s", str(model_file))
            else:
                rospy.logerr("Failed to load ontology from: %s", str(model_file))
                return
        else:
                return


        if self.grounded_configuration is not None:
            rospy.loginfo('grounded_configuration initialized to: %s', self.grounded_configuration)
        else:
            rospy.logwarn('grounded_configuration not found in the param server')

        self.initialized = True
        # Reasoner initialization completed

        self._pub_log = rospy.Publisher('/log', String, queue_size=10)
        self._pub_phasetimes = rospy.Publisher('/phase_times', KeyValue, queue_size=10)

        rospy.loginfo("[RosReasoner] -- Reasoner Initialization Ok")


    def start_timer(self):
        timer_rate =  self.check_and_read_parameter('/reasoning_rate', 2.0)
        timer = rospy.Timer(rospy.Duration(timer_rate), self.timer_cb)


    @staticmethod
    def check_and_read_parameter(param_name, default_value=None):
        """ Checks if a parameter exists and returns its value
            Args:
                    param_name (string): The name of the parameter.
            Returns:
                    The parameter value if it exists, None otherwise.
        """
        if rospy.has_param(str(param_name)):
            rospy.logwarn("Parameter \'%s\' is defined! - Returning %s", str(param_name), rospy.get_param(str(param_name)))
            return rospy.get_param(str(param_name))
        else:
            rospy.logwarn("Parameter \'%s\' not defined! - Returning %s", str(param_name), str(default_value))
            return default_value

    # NOTE REFACTORING: This KB initialization is completely mixed with ROS interfaces, probably libraty should not have an initKB method, but utility methods to update the abox according to incoming information
    # Initializes the KB according to 2 cases:
    # - If there is an Objective individual in the ontology file, the KB is initialized only using the OWL file
    # - If there is no Objective individual, a navigation Objective is create in the KB, with associated NFRs that are read frmo rosparam
    def initKB(self):

        rospy.loginfo('KB initialization:\n \t - Supported QAs: \n \t \t - for Function f_navigate: /nfr_energy, /nfr_safety \n \t - If an Objective instance is not found in the owl file, a default o_navigate is created.' )

        objectives = self.search_objectives()

        # if no objectives in the OWL file, standard navigation objective is assumed
        if objectives == []:
            rospy.loginfo('Creating default Objective o_navigateA with default NFRs')

            obj_navigate = self.get_new_tomasys_objective("o_navigateA", "*f_navigate")

            # Get ontology and tomasys file paths from parameters
            # nfr_energy_value = float(self.check_and_read_parameter('/nfr_energy', 0.5))
            nfr_safety_value = float(self.check_and_read_parameter('/nfr_safety', 0.8))

            # Load NFRs in the KB
            # nfr_energy = self.get_new_tomasys_nrf("nfr_energy", "*energy", nfr_energy_value)
            nfr_safety = self.get_new_tomasys_nrf("nfr_safety", "*safety", nfr_safety_value)

            # Link NFRs to objective
            # obj_navigate.hasNFR.append(nfr_energy)
            obj_navigate.hasNFR.append(nfr_safety)

            # # Function Groundings and Objectives
            self.set_new_grounding(self.grounded_configuration, obj_navigate)

        elif len(objectives) == 1:
            o = objectives[0]
            fd = obtainBestFunctionDesign(o, self.tomasys)
            self.set_new_grounding(fd.name, o)
            rospy.logwarn('Objective, NFRs and initial FG are generated from the OWL file')
        else:
            rospy.logerr('Metacontrol cannot handle more than one Objective in the OWL file (the Root Objective)')

        # For debugging InConsistent ontology errors, save the ontology before reasoning
        self.onto.save(file="tmp_debug.owl", format="rdfxml")


    # MVP: callback for diagnostic msg received from QA Observer
    def callbackDiagnostics(self, msg):
        if self.onto is not None:
            for diagnostic_status in msg.status:
                # 2 types of diagnostics considered: about bindings in error (TODO not implemented yet) or about QAs
                if diagnostic_status.message == "binding error":
                    rospy.loginfo("binding error received")
                    up_binding = self.updateBinding(diagnostic_status.name, diagnostic_status.level)
                    if up_binding == -1:
                        rospy.logwarn("Unkown Function Grounding: %s", diagnostic_name)
                    elif up_binding == 0:
                        rospy.logwarn("Diagnostics message received for %s with level %d, nothing done about it." % (diagnostic_name, diagnostic_level))

                if diagnostic_status.message == "QA status":
                    if diagnostic_status.values[0].key == 'safety':
                        self._pub_log.publish("QA update received, %s value= %s"%(diagnostic_status.values[0].key,diagnostic_status.values[0].value))
                    rospy.logwarn("QA value received for\t{0} \tTYPE: {1}\tVALUE: {2}".format(diagnostic_status.name, diagnostic_status.values[0].key, diagnostic_status.values[0].value))
                    up_qa = self.updateQA(diagnostic_status)
                    if up_qa == -1:
                        rospy.logwarn("QA message refers to a FG not found in the KB, we asume it refers to the current grounded_configuration (1st fg found in the KB)")
                    elif up_qa != 1:
                        # rospy.loginfo("QA value received!\tTYPE: {0}\tVALUE: {1}".format(diagnostic_status.values[0].key, diagnostic_status.values[0].value))
                    # else:
                        rospy.logwarn("Unsupported QA TYPE received: %s ", str(diagnostic_status.values[0].key))

                if diagnostic_status.message == "QA prediction":
                    # rospy.logwarn("QA prediction received for\t{0} \tTYPE: {1}\tVALUE: {2}".format(diagnostic_status.name, diagnostic_status.values[0].key, diagnostic_status.values[0].value))
                    self._pub_log.publish("QA prediction received, %s value= %s"%(diagnostic_status.values[0].key,diagnostic_status.values[0].value))
                    up_qa = self.updateQA_pred(diagnostic_status)
                    if up_qa == -1:
                        rospy.logwarn("QA message refers to a FG not found in the KB, we asume it refers to the current grounded_configuration (1st fg found in the KB)")
                    elif up_qa != 1:
                        # rospy.loginfo("QA prediction received!\tTYPE: {0}\tVALUE: {1}".format(diagnostic_status.values[0].key, diagnostic_status.values[0].value))
                    # else:
                        rospy.logwarn("Unsupported QA TYPE received: %s ", str(diagnostic_status.values[0].key))
    # for MVP with QAs - request the FD.name to reconfigure to
    def request_configuration(self, fd):
        # rospy.logwarn_throttle(1., 'New Configuration requested: {}'.format(fd.name))

        goal = MvpReconfigurationGoal()
        goal.desired_configuration_name = fd.name

        action_server_up = self.rosgraph_manipulator_client.wait_for_server()
        if not action_server_up:
            rospy.logerr("Action server not found, Aborting reconfiguration!")
            return
        self.rosgraph_manipulator_client.send_goal(goal)
        goal_completed = self.rosgraph_manipulator_client.wait_for_result()
        if not goal_completed:
            rospy.logwarn("result not found")
            return

        result = self.rosgraph_manipulator_client.get_result().result
        # rospy.loginfo('Result: {}'.format(result) )
        return result

    ## main metacontrol loop
    def timer_cb(self, event):

        # rospy.loginfo('Entered timer_cb for metacontrol reasoning')
        # rospy.loginfo('  >> Started MAPE-K ** Analysis (ontological reasoning) **')

        # EXEC REASONING to update ontology with inferences
        self._pub_log.publish("start analyse fase")
        phase_start_time = rospy.get_time()
        if not self.perform_reasoning():
            # rospy.loginfo('     >> Finished ontological reasoning)')
        # else:
            rospy.logerr("Reasoning error")

        # PRINT system status
        print_ontology_status(self.tomasys)
            
        # EVALUATE functional hierarchy (objectives statuses) (MAPE - Analysis)
        objectives_internal_error = evaluateObjectives(self.tomasys)
        check_desired_config = False
        if not objectives_internal_error:
            rospy.loginfo("No Objectives in status ERROR: no adaptation is needed")
            self._pub_log.publish("end analyse fase")
            self._pub_phasetimes.publish(KeyValue("analyse",str(rospy.get_time() - phase_start_time)))

            if self.modification == 2:
                if self.grounded_configuration == self.desired_configuration:
                    return
                else:
                    print(self.grounded_configuration)
                    print(self.desired_configuration)
                    objectives_internal_error = [self.onto.o_navigateA]
                    check_desired_config = True
            else:
                return        
            # rospy.loginfo('  >> Finished MAPE-K ** ANALYSIS **')
            # rospy.loginfo('Exited timer_cb for metacontrol reasoning')
        elif len(objectives_internal_error) > 1 :
            # rospy.logerr("- More than 1 objectives in error, case not supported yet.")
            # rospy.loginfo('  >> Finished MAPE-K ** ANALYSIS **')
            # rospy.loginfo('Exited timer_cb for metacontrol reasoning')
            return
        else:
            self._pub_log.publish("nfr violation found")
            self._pub_log.publish("end analyse fase")
            self._pub_phasetimes.publish(KeyValue("analyse",str(rospy.get_time() - phase_start_time)))

            rospy.logwarn("Objectives in status ERROR: {}".format([o.name for o in objectives_internal_error]) )
            # rospy.loginfo('  >> Finished MAPE-K ** ANALYSIS **')

        # Update kb with predicted QA values
        # rospy.loginfo('  >> Request for QA updates **')
        try:
            self._pub_log.publish("request qa predictions")
            phase_start_time = rospy.get_time()
            req_qa_updates = rospy.ServiceProxy('/qa_pred_update', QAPredictions)
            # rospy.wait_for_service('/qa_pred_update')
            resp = req_qa_updates("")
            self.updateQA_pred(resp.values)
            # rospy.loginfo("QA update request send")
            self._pub_log.publish("qa predictions processed")
            self._pub_phasetimes.publish(KeyValue("qa_update",str(rospy.get_time() - phase_start_time)))
        except Exception as exc:
            print(exc)
            rospy.loginfo('/qa_pred_update service not available')

        # ADAPT MAPE -Plan & Execute
        rospy.loginfo('  >> Started MAPE-K ** PLAN adaptation **')
        if check_desired_config:
            self._pub_log.publish("start plan fase to check desired configuration")
            phase_start_time = rospy.get_time()
            o = objectives_internal_error[0]
            fd = test_desired_config(o, self.tomasys, self.desired_configuration)
            if not fd:
                return
            self._pub_log.publish("end plan fase")
            self._pub_phasetimes.publish(KeyValue("plan",str(rospy.get_time() - phase_start_time)))
        else:
            self._pub_log.publish("start plan fase")
            phase_start_time = rospy.get_time()
            o = objectives_internal_error[0]
            # rospy.loginfo("=> Reasoner searches FD for objective: {}".format(o.name) )
            fd = obtainBestFunctionDesign(o, self.tomasys, self.modification)
            if not fd:
                rospy.logerr(
                    "No FD found to solve Objective {}".format(o.name)) # for DEBUGGING in csv
                rospy.loginfo('Exited timer_cb for metacontrol reasoning')
                return
            # rospy.loginfo('  >> Finished MAPE-K ** Plan adaptation **')
            self._pub_log.publish("end plan fase")
            self._pub_phasetimes.publish(KeyValue("plan",str(rospy.get_time() - phase_start_time)))

        # request new configuration
        # rospy.loginfo('  >> Started MAPE-K ** EXECUTION **')
        self._pub_log.publish("start execution fase")
        phase_start_time = rospy.get_time()
        result = self.request_configuration(fd)
        # rospy.loginfo('  >> Finished MAPE-K ** EXECUTION **')
        # Process adaptation feedback to update KB:
        if result == 1: # reconfiguration executed ok
            self._pub_log.publish("end execution fase")
            self._pub_phasetimes.publish(KeyValue("execution",str(rospy.get_time() - phase_start_time)))
            rospy.logwarn("= RECONFIGURATION SUCCEEDED =") # for DEBUGGING in csv
            # updates the ontology according to the result of the adaptation action - destroy fg for Obj and create the newly grounded one
            self.grounded_configuration = self.set_new_grounding(fd.name, o) # Set new grounded_configuration
            resetKBstatuses(self.tomasys)
        elif result == -1:
            rospy.logerr("= RECONFIGURATION UNKNOWN =") # for DEBUGGING in csv
        else:
            rospy.logerr("= RECONFIGURATION FAILED =") # for DEBUGGING in csv
        rospy.loginfo('Exited timer_cb for metacontrol reasoning')
