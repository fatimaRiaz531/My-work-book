import React, { useState } from 'react';
import { useAuth } from '../Auth/AuthProvider';
import subagentManager from '../../lib/subagents/SubagentManager';

const PersonalizationButton = ({ content, contentType = 'chapter' }) => {
  const { user } = useAuth();
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState(content);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handlePersonalize = async () => {
    if (!user) {
      alert('Please log in to access personalization features');
      return;
    }

    // Check if content is already personalized
    if (isPersonalized) {
      setPersonalizedContent(content);
      setIsPersonalized(false);
      setError('');
      return;
    }

    setLoading(true);
    setError('');

    try {
      // Use the PersonalizationSubagent for content personalization
      const result = await subagentManager.executeSubagent('PersonalizationSubagent', {
        content,
        userBackground: {
          softwareBackground: user.softwareBackground,
          hardwareBackground: user.hardwareBackground,
          name: user.name
        },
        contentType
      });

      if (result.success) {
        setPersonalizedContent(result.result.personalizedContent);
        setIsPersonalized(true);
      } else {
        throw new Error(result.error || 'Personalization failed');
      }
    } catch (error) {
      console.error('Personalization error:', error);
      setError('Personalization failed. Please try again.');
      alert('Error personalizing content. Using original content.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="personalization-container">
      <button
        onClick={handlePersonalize}
        className={`personalize-button ${isPersonalized ? 'active' : ''} ${loading ? 'loading' : ''}`}
        disabled={!user || loading}
        title={user ?
          (isPersonalized ? 'Revert to original content' : 'Personalize this content') :
          'Login to personalize content'
        }
      >
        {loading ? 'Personalizing...' :
         isPersonalized ? ' revert to original' : ' personalize content'}
      </button>

      {error && <div className="error-message">{error}</div>}

      <div className="content-display">
        {personalizedContent}
      </div>
    </div>
  );
};

export default PersonalizationButton;