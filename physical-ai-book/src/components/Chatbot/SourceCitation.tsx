/**
 * SourceCitation Component
 *
 * Displays citation information for a source chunk.
 */

import React from 'react';
import type { SourceCitation as SourceCitationType } from './types';

interface SourceCitationProps {
  source: SourceCitationType;
  index: number;
}

const SourceCitation: React.FC<SourceCitationProps> = ({ source, index }) => {
  return (
    <div className="source-citation">
      <div className="citation-header">
        <strong>Source {index}:</strong> {source.section}
        {source.page && <span> (Page {source.page})</span>}
      </div>
      {source.text_preview && (
        <div className="citation-preview">
          {source.text_preview}...
        </div>
      )}
      <div className="citation-score">
        Relevance: {(source.similarity_score * 100).toFixed(1)}%
      </div>
    </div>
  );
};

export default SourceCitation;
