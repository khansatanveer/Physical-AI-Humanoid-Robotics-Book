// chatbot/hooks/useTextSelection.js
import { useState, useEffect, useCallback } from 'react';

const useTextSelection = () => {
  const [selectedText, setSelectedText] = useState('');
  const [selectionPosition, setSelectionPosition] = useState(null);

  // Function to get currently selected text
  const getSelectedText = useCallback(() => {
    const selection = window.getSelection ? window.getSelection() : document.selection;
    if (selection) {
      const text = selection.toString().trim();
      return text;
    }
    return '';
  }, []);

  // Function to get selection coordinates
  const getSelectionPosition = useCallback(() => {
    const selection = window.getSelection ? window.getSelection() : document.selection;
    if (selection && selection.rangeCount > 0) {
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();
      return {
        x: rect.left + window.scrollX,
        y: rect.top + window.scrollY,
        width: rect.width,
        height: rect.height
      };
    }
    return null;
  }, []);

  // Handle text selection
  const handleSelection = useCallback(() => {
    const text = getSelectedText();
    const position = getSelectionPosition();

    // Only update if text has actually changed
    if (text !== selectedText) {
      setSelectedText(text);
      setSelectionPosition(position);
    }
  }, [getSelectedText, getSelectionPosition, selectedText]);

  // Set up event listeners
  useEffect(() => {
    // Add event listeners for text selection
    const handleMouseUp = () => handleSelection();
    const handleKeyUp = (e) => {
      // Only respond to key events that could change selection (like Shift+Arrow)
      if (e.key.startsWith('Arrow') || e.key === 'Shift') {
        setTimeout(handleSelection, 0); // Use timeout to ensure selection is updated
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('keyup', handleKeyUp);

    // Cleanup event listeners
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('keyup', handleKeyUp);
    };
  }, [handleSelection]);

  // Clear selection when needed
  const clearSelection = useCallback(() => {
    if (window.getSelection) {
      window.getSelection().removeAllRanges();
    } else if (document.selection) {
      document.selection.empty();
    }
    setSelectedText('');
    setSelectionPosition(null);
  }, []);

  // Get current selection without updating state
  const getCurrentSelection = useCallback(() => {
    return {
      text: getSelectedText(),
      position: getSelectionPosition()
    };
  }, [getSelectedText, getSelectionPosition]);

  return {
    selectedText,
    selectionPosition,
    getSelectedText,
    getCurrentSelection,
    clearSelection,
    updateSelection: handleSelection
  };
};

export default useTextSelection;