import React from 'react';
import clsx from 'clsx';
import styles from './PrerequisiteIndicator.module.css';

interface PrerequisiteIndicatorProps {
  prerequisites: string[];
  completed?: string[];
}

const PrerequisiteIndicator: React.FC<PrerequisiteIndicatorProps> = ({
  prerequisites,
  completed = []
}) => {
  const completedCount = prerequisites.filter(prereq =>
    completed.includes(prereq)
  ).length;

  const isSatisfied = completedCount >= prerequisites.length;

  return (
    <div className={clsx('margin-bottom--lg', styles.prereqIndicator)}>
      <div className={styles.prereqHeader}>
        <h4 className={styles.prereqTitle}>
          {isSatisfied ? '✅ Prerequisites Satisfied' : '📋 Prerequisites'}
        </h4>
      </div>

      <div className={styles.prereqBody}>
        <div className={styles.prereqProgress}>
          <div className={styles.progressBar}>
            <div
              className={styles.progressFill}
              style={{ width: `${(completedCount / prerequisites.length) * 100}%` }}
            ></div>
          </div>
          <div className={styles.progressText}>
            {completedCount} of {prerequisites.length} completed
          </div>
        </div>

        <ul className={styles.prereqList}>
          {prerequisites.map((prereq, index) => {
            const isCompleted = completed.includes(prereq);
            return (
              <li
                key={index}
                className={clsx(styles.prereqItem, {
                  [styles.completed]: isCompleted,
                  [styles.incomplete]: !isCompleted
                })}
              >
                {isCompleted ? '✅' : '⭕'} {prereq}
              </li>
            );
          })}
        </ul>
      </div>
    </div>
  );
};

export default PrerequisiteIndicator;