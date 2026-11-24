## ADDED Requirements

### Requirement: Xacro CAN timing parameters
The hardware SHALL accept optional can_bitrate and can_sample_point from hardware parameters without assigning defaults. It SHALL validate each supplied value during on_init: positive int-range bitrate and finite sample point in (0, 1].

#### Scenario: Missing parameters
- **WHEN** neither parameter is supplied
- **THEN** the hardware passes an empty extend object without a buses entry and does not request a bus update, leaving defaults or existing settings to the driver

#### Scenario: Only one parameter supplied
- **WHEN** only can_bitrate or can_sample_point is supplied
- **THEN** only that field is forwarded and the hardware does not fill in the other field

#### Scenario: Empty xacro arguments
- **WHEN** the optional CAN xacro arguments are omitted or left empty
- **THEN** the corresponding hardware parameters are not generated

#### Scenario: Explicit timing
- **WHEN** valid values are supplied through xacro
- **THEN** the hardware uses those values for the selected bus

#### Scenario: Invalid value
- **WHEN** either supplied value is empty, malformed, out of range, or non-finite
- **THEN** on_init returns ERROR before configuring an adapter

### Requirement: Scoped configuration during configure
The hardware SHALL pass only explicitly supplied bitrate and/or sample_point under the selected extend.buses entry to MakeAdapter during on_configure. It SHALL preserve the existing device interfaces and runtime I/O.

#### Scenario: Selected EtherCAT bus
- **WHEN** the adapter is Ethercat or EthercatIGH
- **THEN** the entry uses slave_idx:bus_idx and does not configure any other bus

#### Scenario: Direct CAN bus
- **WHEN** the adapter is Can
- **THEN** the entry uses bus 0 and applies when the adapter is first created

#### Scenario: Adapter reuse
- **WHEN** a shared EtherCAT adapter already exists and CAN parameters are supplied
- **THEN** MakeAdapter routes the selected bus settings through the driver's existing update mechanism without a second update call

#### Scenario: Adapter reuse without timing
- **WHEN** a shared adapter already exists and neither CAN parameter is supplied
- **THEN** the hardware reuses the adapter without invoking its bus update mechanism or overwriting existing settings

#### Scenario: Driver rejects configuration
- **WHEN** configuration throws or no device is returned
- **THEN** on_configure reports the device, bus and failure and returns ERROR without activation

### Requirement: Document bus-wide configuration consistency
The README SHALL explain that one physical bus has one consistent CAN configuration for all sensors, motors and other CAN or CAN FD devices. It SHALL describe driver-owned configuration: on adapters supporting updates, the last successful explicit value for each field takes effect while omitted fields retain current values. The hardware SHALL not implement cross-plugin configuration checks.

#### Scenario: Shared bus
- **WHEN** multiple devices share a physical bus
- **THEN** the documentation requires consistent nominal bitrate and sample point, and consistent FD data-phase timing where applicable
- **AND** it explains that one consumer may supply the configuration while others omit it, and repeated explicit configuration is resolved by the driver

#### Scenario: SocketCAN configuration changes
- **WHEN** settings for an existing SocketCAN adapter change
- **THEN** the documentation requires a process restart and consistent configuration from every bus consumer
