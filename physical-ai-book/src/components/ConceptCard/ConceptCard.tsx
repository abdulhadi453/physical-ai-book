import React from 'react';
import clsx from 'clsx';
import styles from './ConceptCard.module.css';

interface ConceptCardProps {
  title: string;
  description: string;
  examples?: string[];
  keyPoints?: string[];
}

const ConceptCard: React.FC<ConceptCardProps> = ({
  title,
  description,
  examples,
  keyPoints
}) => {
  return (
    <div className={clsx('margin-bottom--lg', styles.conceptCard)}>
      <div className={styles.conceptHeader}>
        <h3 className={styles.conceptTitle}>💡 {title}</h3>
      </div>

      <div className={styles.conceptBody}>
        <div className={styles.conceptSection}>
          <p>{description}</p>
        </div>

        {examples && examples.length > 0 && (
          <div className={styles.conceptSection}>
            <h4>Examples:</h4>
            <ul>
              {examples.map((example, index) => (
                <li key={index}>{example}</li>
              ))}
            </ul>
          </div>
        )}

        {keyPoints && keyPoints.length > 0 && (
          <div className={styles.conceptSection}>
            <h4>Key Points:</h4>
            <ul>
              {keyPoints.map((point, index) => (
                <li key={index}>{point}</li>
              ))}
            </ul>
          </div>
        )}
      </div>
    </div>
  );
};

export default ConceptCard;