// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  projectSidebar: [
    'intro',
    'project-map',
    'robot-overview',
    'system-architecture',
    {
      type: 'category',
      label: 'Mechanical',
      items: [
        'mechanical/overview',
        'mechanical/torque-calculations',
        'mechanical/leg-design',
        'mechanical/cad-and-fabrication',
      ],
    },
    {
      type: 'category',
      label: 'Electronics and Control',
      items: [
        'electronics-control/overview',
        'electronics-control/power-system',
        'electronics-control/motor-controllers',
        'electronics-control/sensors',
        'electronics-control/kinematics',
      ],
    },
    {
      type: 'category',
      label: 'Assembly',
      items: [
        'assembly/mechanical-assembly',
        'assembly/wiring',
        'assembly/motor-controller-setup',
        'assembly/pre-power-checks',
      ],
    },
    {
      type: 'category',
      label: 'Testing',
      items: [
        'testing/first-power-on',
        'testing/motor-tests',
        'testing/joint-calibration',
        'testing/walking-tests',
        'testing/troubleshooting',
        'testing/known-issues',
      ],
    },
    {
      type: 'category',
      label: 'Software and Simulation',
      items: [
        'software-simulation/code-map',
        'software-simulation/setup',
        'software-simulation/test-scripts',
        'software-simulation/logging-debugging',
      ],
    },
    {
      type: 'category',
      label: 'Theory / Background',
      items: [
        {
          type: 'category',
          label: 'Mechanical',
          items: [
            'theory/mechanical/actuation-survey',
            'theory/mechanical/motors',
            'theory/mechanical/gearboxes',
            'theory/mechanical/joint-torque-sizing-theory',
          ],
        },
        {
          type: 'category',
          label: 'Electronics and Control',
          items: [
            'theory/electronics-control/power-basics',
            'theory/electronics-control/pid-control',
            'theory/electronics-control/bldc-motor-control',
            'theory/electronics-control/foc-control',
            'theory/electronics-control/encoders-and-sensing',
            'theory/electronics-control/leg-kinematics',
            'theory/electronics-control/gaits',
            'theory/electronics-control/sim-to-real-notes',
          ],
        },
      ],
    },
    'roadmap',
    'contribute',
  ],
};

module.exports = sidebars;
