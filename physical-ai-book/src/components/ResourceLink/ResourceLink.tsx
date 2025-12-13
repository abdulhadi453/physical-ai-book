import React from 'react';
import clsx from 'clsx';
import styles from './ResourceLink.module.css';

interface ResourceLinkProps {
  title: string;
  url: string;
  type: 'article' | 'video' | 'code' | 'tool' | 'book';
  description: string;
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
}

const ResourceLink: React.FC<ResourceLinkProps> = ({
  title,
  url,
  type,
  description,
  difficulty
}) => {
  const getTypeIcon = (type: string) => {
    switch (type) {
      case 'article': return '📄';
      case 'video': return '🎬';
      case 'code': return '💻';
      case 'tool': return '⚙️';
      case 'book': return '📚';
      default: return '🔗';
    }
  };

  const getDifficultyColor = (difficulty: string | undefined) => {
    switch (difficulty) {
      case 'beginner': return '#4285f4';
      case 'intermediate': return '#fbbc04';
      case 'advanced': return '#ea4335';
      default: return '#9aa0a6';
    }
  };

  return (
    <div className={clsx('margin-bottom--md', styles.resourceLink)}>
      <div className={styles.resourceHeader}>
        <h4 className={styles.resourceTitle}>
          {getTypeIcon(type)} <a href={url} target="_blank" rel="noopener noreferrer">{title}</a>
        </h4>
        {difficulty && (
          <span
            className={styles.difficultyBadge}
            style={{ backgroundColor: getDifficultyColor(difficulty) }}
          >
            {difficulty}
          </span>
        )}
      </div>
      <div className={styles.resourceDescription}>
        {description}
      </div>
    </div>
  );
};

export default ResourceLink;