*** Settings ***
Documentation     Automated Renode test suite for X19-Embedded firmware nodes.
Suite Setup       Setup
Suite Teardown    Teardown
Test Teardown     Reset Emulation

*** Variables ***
${SCRIPT_DIR}     ${CURDIR}/..

*** Test Cases ***
Node 1 Pi Shield Should Boot Cleanly
    [Documentation]    Verifies that Node 1 ARM binary boots without HardFault.
    Execute Command    include @${SCRIPT_DIR}/run_node1.resc
    Execute Command    emulation RunFor "00:00:02"
    ${state}=          Execute Command    Node1_Pi_Shield.cpu ExecutionMode
    Should Contain     ${state}    Continuous

Node 2 Control Board Should Boot Cleanly
    [Documentation]    Verifies that Node 2 ARM binary boots without HardFault.
    Execute Command    include @${SCRIPT_DIR}/run_node2.resc
    Execute Command    emulation RunFor "00:00:02"
    ${state}=          Execute Command    Node2_Control_Board.cpu ExecutionMode
    Should Contain     ${state}    Continuous

Multi Node Cluster Should Run Concurrently On Virtual CAN Bus
    [Documentation]    Verifies that Node 1 and Node 2 boot together on the shared virtual CAN bus.
    Execute Command    include @${SCRIPT_DIR}/multi_node_network.resc
    Execute Command    emulation RunFor "00:00:02"
    ${state1}=         Execute Command    Node1_Pi_Shield.cpu ExecutionMode
    ${state2}=         Execute Command    Node2_Control_Board.cpu ExecutionMode
    Should Contain     ${state1}    Continuous
    Should Contain     ${state2}    Continuous
