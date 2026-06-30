// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  projectSidebar: [
    'intro',
    'project-map',
    {
      type: 'category',
      label: 'Mechanical',
      items: [
        'mechanical/overview',
        'mechanical/actuation-survey',
        'mechanical/joint-torque-sizing',
        'mechanical/motors',
        'mechanical/gearboxes',
        'mechanical/our-robot',
      ],
    },
    {
      type: 'category',
      label: 'Electronics',
      items: ['electronics/overview'],
    },
    {
      type: 'category',
      label: 'Software',
      items: ['software/overview'],
    },
    {
      type: 'category',
      label: 'Controls',
      items: ['controls/overview'],
    },
    {
      type: 'category',
      label: 'Simulation',
      items: ['simulation/genesis'],
    },
    {
      type: 'category',
      label: 'Assembly',
      items: ['assembly/overview'],
    },
    {
      type: 'category',
      label: 'Bring-up',
      items: ['bringup/overview'],
    },
    'troubleshooting',
    'roadmap',
    'contribute',
  ],
};

module.exports = sidebars;
