// chatbot/components/ModeSelector.jsx
import React from 'react';
import PropTypes from 'prop-types';

const ModeSelector = ({ currentMode, onModeChange }) => {
  return (
    <div className="mode-selector">
      <button
        className={`mode-button ${currentMode === 'global' ? 'active' : ''}`}
        onClick={() => onModeChange('global')}
        aria-label="Switch to global book mode"
        title="Ask questions about the entire book/documentation set"
      >
        Global Book
      </button>
      <button
        className={`mode-button ${currentMode === 'selection' ? 'active' : ''}`}
        onClick={() => onModeChange('selection')}
        aria-label="Switch to selected text mode"
        title="Ask questions about selected text only"
      >
        Selected Text
      </button>
    </div>
  );
};

ModeSelector.propTypes = {
  currentMode: PropTypes.oneOf(['global', 'selection']).isRequired,
  onModeChange: PropTypes.func.isRequired,
};

export default ModeSelector;