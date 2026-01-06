/**
 * ChatMessage Component
 *
 * Displays individual messages in the chat (user questions or assistant responses).
 */

import React from 'react';
import type { ChatMessage as ChatMessageType } from './types';
import SourceCitation from './SourceCitation';

interface ChatMessageProps {
  message: ChatMessageType;
}

const ChatMessage: React.FC<ChatMessageProps> = ({ message }) => {
  const isUser = message.type === 'user';

  return (
    <div className={`chat-message ${isUser ? 'user-message' : 'assistant-message'}`}>
      <div className="message-header">
        <span className="message-sender">
          {isUser ? 'You' : 'Assistant'}
        </span>
        <span className="message-timestamp">
          {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </span>
      </div>
      <div className="message-content">
        {message.content}
      </div>
      {!isUser && message.sources && message.sources.length > 0 && (
        <div className="message-sources">
          <h4>Sources:</h4>
          {message.sources.map((source, index) => (
            <SourceCitation key={source.chunk_id} source={source} index={index + 1} />
          ))}
        </div>
      )}
    </div>
  );
};

export default ChatMessage;
