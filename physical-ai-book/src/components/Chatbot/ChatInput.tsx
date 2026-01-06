/**
 * ChatInput Component
 *
 * Provides input field and submit button for user questions.
 */

import React from 'react';

interface ChatInputProps {
  value: string;
  onChange: (value: string) => void;
  onSubmit: () => void;
  disabled: boolean;
  placeholder?: string;
}

const ChatInput: React.FC<ChatInputProps> = ({
  value,
  onChange,
  onSubmit,
  disabled,
  placeholder = "Ask a question about the Physical AI book..."
}) => {
  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !disabled) {
      onSubmit();
    }
  };

  return (
    <div className="chat-input-container">
      <input
        type="text"
        className="chat-input"
        value={value}
        onChange={(e) => onChange(e.target.value)}
        onKeyPress={handleKeyPress}
        disabled={disabled}
        placeholder={placeholder}
        aria-label="Question input"
      />
      <button
        className="chat-submit-button"
        onClick={onSubmit}
        disabled={disabled || !value.trim()}
        aria-label="Submit question"
      >
        Send
      </button>
    </div>
  );
};

export default ChatInput;
