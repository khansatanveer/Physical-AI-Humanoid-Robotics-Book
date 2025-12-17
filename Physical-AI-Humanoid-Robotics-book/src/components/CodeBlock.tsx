import React from 'react';

type CodeBlockProps = {
  children: React.ReactNode;
  title?: string;
  language?: string;
  showLineNumbers?: boolean;
};

const CodeBlock: React.FC<CodeBlockProps> = ({
  children,
  title,
  language = 'text',
  showLineNumbers = false
}) => {
  const codeString = React.Children.toArray(children).join('').trim();

  return (
    <div className="code-block-wrapper">
      {title && (
        <div className="code-block-header">
          <span className="code-block-title">{title}</span>
        </div>
      )}
      <pre className={`language-${language}`}>
        <code className={`language-${language}`}>
          {codeString}
        </code>
      </pre>
    </div>
  );
};

export default CodeBlock;