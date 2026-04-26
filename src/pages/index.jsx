import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './index.module.css';

const focusAreas = [
  {
    title: 'Mechanical Platform',
    copy: 'Frame, legs, joints, actuators, manufacturing notes, CAD links, and assembly order.',
  },
  {
    title: 'Electronics Stack',
    copy: 'Power distribution, motor drivers, sensors, wiring, connectors, and bring-up checks.',
  },
  {
    title: 'Locomotion Software',
    copy: 'Control scripts, simulation, gait experiments, logs, and the current code map.',
  },
];

const milestones = [
  'Preserve the full technical state before graduation',
  'Make onboarding possible for freshers without oral history',
  'Turn experiments, failures, and next steps into reusable knowledge',
];

function ProjectVisual() {
  const teaserUrl = useBaseUrl('/img/genesis-teaser.png');

  return (
    <div className={styles.visualShell} aria-label="Quadruped project visual">
      <img
        className={styles.teaserImage}
        src={teaserUrl}
        alt="Robotics simulation and rendering collage"
      />
      <div className={styles.robotPlate}>
        <div className={styles.robotBody}>
          <span />
          <span />
          <span />
        </div>
        <div className={styles.legFront}>
          <i />
          <b />
        </div>
        <div className={styles.legBack}>
          <i />
          <b />
        </div>
        <div className={styles.legRear}>
          <i />
          <b />
        </div>
        <div className={styles.floorLine} />
      </div>
    </div>
  );
}

function HomepageHeader() {
  return (
    <header className={styles.hero}>
      <div className={styles.heroInner}>
        <section className={styles.heroCopy}>
          <p className={styles.kicker}>ERC-BPGC Robotics</p>
          <Heading as="h1" className={styles.heroTitle}>
            ERC Quadruped
          </Heading>
          <p className={styles.heroSubtitle}>
            A living engineering handbook for a student-built quadruped: CAD,
            electronics, controls, simulation, experiments, and the decisions
            that made the robot what it is.
          </p>
          <div className={styles.heroActions}>
            <Link className="button button--primary button--lg" to="/docs/intro">
              Start reading
            </Link>
            <Link className="button button--secondary button--lg" to="/docs/project-map">
              View project map
            </Link>
          </div>
        </section>
        <ProjectVisual />
      </div>
    </header>
  );
}

function FocusAreas() {
  return (
    <section className={styles.section}>
      <div className={styles.sectionHeader}>
        <p className={styles.kicker}>Documentation System</p>
        <Heading as="h2">Built for handoff, not just show</Heading>
      </div>
      <div className={styles.focusGrid}>
        {focusAreas.map((area) => (
          <article className={styles.focusCard} key={area.title}>
            <h3>{area.title}</h3>
            <p>{area.copy}</p>
          </article>
        ))}
      </div>
    </section>
  );
}

function MissionStrip() {
  return (
    <section className={clsx(styles.section, styles.missionStrip)}>
      <div>
        <p className={styles.kicker}>Why this exists</p>
        <Heading as="h2">So the next team starts from what we learned.</Heading>
      </div>
      <ul>
        {milestones.map((item) => (
          <li key={item}>{item}</li>
        ))}
      </ul>
    </section>
  );
}

export default function Home() {
  return (
    <Layout
      title="ERC Quadruped"
      description="Documentation for the ERC-BPGC quadruped robotics project"
    >
      <HomepageHeader />
      <main>
        <FocusAreas />
        <MissionStrip />
      </main>
    </Layout>
  );
}
