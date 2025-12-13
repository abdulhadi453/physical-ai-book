import React from 'react';
import clsx from 'clsx';
import styles from './SummarySection.module.css';

interface SummarySectionProps {
  title?: string;
  keyTakeaways: string[];
  nextSteps?: string[];
}

const SummarySection: React.FC<SummarySectionProps> = ({
  title = 'Summary',
  keyTakeaways,
  nextSteps
}) => {
  return (
    <div className={clsx('margin-bottom--lg', styles.summarySection)}>
      <div className={styles.summaryHeader}>
        <h3 className={styles.summaryTitle}>📋 {title}</h3>
      </div>

      <div className={styles.summaryBody}>
        <div className={styles.summarySection}>
          <h4>Key Takeaways:</h4>
          <ul>
            {keyTakeaways.map((takeaway, index) => (
              <li key={index}>{takeaway}</li>
            ))}
          </ul>
        </div>

        {nextSteps && nextSteps.length > 0 && (
          <div className={styles.summarySection}>
            <h4>Next Steps:</h4>
            <ul>
              {nextSteps.map((step, index) => (
                <li key={index}>{step}</li>
              ))}
            </ul>
          </div>
        )}
      </div>
    </div>
  );
};

export default SummarySection;