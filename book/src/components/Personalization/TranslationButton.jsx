import React, { useState } from 'react';
import { useAuth } from '../Auth/AuthProvider';
import subagentManager from '../../lib/subagents/SubagentManager';

const TranslationButton = ({ content, contentType = 'chapter' }) => {
  const { user } = useAuth();
  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState(content);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleTranslate = async () => {
    if (!user) {
      alert('Please log in to access translation features');
      return;
    }

    // Check if content is already translated
    if (isTranslated) {
      setTranslatedContent(content);
      setIsTranslated(false);
      setError('');
      return;
    }

    setLoading(true);
    setError('');

    try {
      // Use the TranslationSubagent for Urdu translation
      const result = await subagentManager.executeSubagent('TranslationSubagent', {
        content,
        targetLanguage: 'ur',
        sourceLanguage: 'en'
      });

      if (result.success) {
        setTranslatedContent(result.result.translatedContent);
        setIsTranslated(true);
      } else {
        throw new Error(result.error || 'Translation failed');
      }
    } catch (error) {
      console.error('Translation error:', error);
      setError('Translation failed. Please try again.');
      alert('Error translating content. Using original content.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="translation-container">
      <button
        onClick={handleTranslate}
        className={`translate-button ${isTranslated ? 'active' : ''} ${loading ? 'loading' : ''}`}
        disabled={!user || loading}
        title={user ?
          (isTranslated ? 'Revert to original content' : 'Translate to Urdu') :
          'Login to translate content'
        }
      >
        {loading ? 'Translating...' :
         isTranslated ? ' revert to English' : ' translate to Urdu'}
      </button>

      {error && <div className="error-message">{error}</div>}

      <div className="content-display">
        {translatedContent}
      </div>
    </div>
  );
};

export default TranslationButton;