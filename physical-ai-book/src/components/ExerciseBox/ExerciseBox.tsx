import React from 'react';
import clsx from 'clsx';
import styles from './ExerciseBox.module.css';

interface ExerciseBoxProps {
  title: string;
  instructions: string;
  expectedOutcome?: string;
  toolsRequired?: string[];
  troubleshootingTips?: string[];
}

const ExerciseBox: React.FC<ExerciseBoxProps> = ({
  title,
  instructions,
  expectedOutcome,
  toolsRequired,
  troubleshootingTips
}) => {
  return (
    <div className={clsx('margin-bottom--lg', styles.exerciseBox)}>
      <div className={styles.exerciseHeader}>
        <h3 className={styles.exerciseTitle}>🔧 {title}</h3>
      </div>

      <div className={styles.exerciseBody}>
        <div className={styles.exerciseSection}>
          <h4>Instructions:</h4>
          <p>{instructions}</p>
        </div>

        {expectedOutcome && (
          <div className={styles.exerciseSection}>
            <h4>Expected Outcome:</h4>
            <p>{expectedOutcome}</p>
          </div>
        )}

        {toolsRequired && toolsRequired.length > 0 && (
          <div className={styles.exerciseSection}>
            <h4>Tools Required:</h4>
            <ul>
              {toolsRequired.map((tool, index) => (
                <li key={index}>{tool}</li>
              ))}
            </ul>
          </div>
        )}

        {troubleshootingTips && troubleshootingTips.length > 0 && (
          <div className={styles.exerciseSection}>
            <h4>Troubleshooting Tips:</h4>
            <ul>
              {troubleshootingTips.map((tip, index) => (
                <li key={index}>{tip}</li>
              ))}
            </ul>
          </div>
        )}
      </div>
    </div>
  );
};

export default ExerciseBox;